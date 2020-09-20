import numpy as np
import tf
import pickle
import argparse
import sys
import os 
import threading
import time
import cv2
import json
import signal
import math
import unicodedata
import re

from datetime import datetime

from scipy.spatial.distance import cdist, pdist
from scipy.cluster.hierarchy import fclusterdata
from scipy.optimize import linear_sum_assignment


import message_filters
import rospy
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import String, Bool
import sensor_msgs.point_cloud2 as pc2
import actionlib
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
from intersection_mapper.msg import Routes2
from intersection_mapper.srv import *
from my_axis_camera.msg import Axis
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from door_navigation.srv import *
from google.cloud import vision

bridge = CvBridge()
camera_model = PinholeCameraModel()
roihist = pickle.load(open('/home/tilyevsk/robot-slang/husky-custom/src/door_detection/src/roihist.pkl', 'rb'))

#set up transformation matrices
trans = [-0.03596812, 0.27371754, -0.08881337]
rot = [0.544521193495, -0.512626565799, 0.452427950113, 0.485818509146]
rot_euler = tf.transformations.euler_from_quaternion(rot)
rot = tf.transformations.quaternion_from_euler(2.68, -1.42, -1.1)
trans = tuple(trans) + ( 1,  )
rotationMatrix = tf.transformations.quaternion_matrix( rot )
rotationMatrix[ :, 3 ] = trans

##############################################################
# initialize variables
##############################################################

all_doors = []
master_door_list = np.array([])
master_door_sides_list = np.array([])
master_door_info = [master_door_list, master_door_sides_list]
doors_checked = {'left':[], 'right':[], 'all':[]}
robot_loc = [None, None, None]
forward = None
run = False
frozen = True
axis_state = {}
new_update = False
fresh_image = False
image_global = None
num_forward_updates = 0
forward_missing = 0
robot_is_stuck = False
monitor_if_stuck = False
robot_stuck_timeout = 20.0

######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	print(msg)
	output_log.write(datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + ' ' + msg + '\n')

######################################################################################
# update the state of the camera
######################################################################################
def update_state(data):
	global axis_state, new_update
	axis_state['pan']        = data.pan
	axis_state['tilt']       = data.tilt
	axis_state['zoom']       = data.zoom
	axis_state['brightness'] = data.brightness
	axis_state['focus']      = data.focus
	axis_state['iris']       = data.iris
	axis_state['autofocus']  = data.autofocus
	new_update               = True

#########################################################
# go to absolute PTZ position
# PARAM			RANGE						DESCRIPTION
# pan 			[-180,180] or [0.0, 1.0]	Pan value in degrees (or percent in absolute mode)
# tilt 			[-90,90] or [0.0, 1.0] 		Tilt value in degrees (or percent in absolute mode)
# brightness 	[1,9999]					Brightness
# focus 		[1,9999]					Focus
# autofocus		True/False 					Autofocus
#########################################################
def set_pt(pan, tilt):
	# setup message
	msg = Axis()
	msg.cmd_type   = 'pt'
	msg.pan        = pan
	msg.tilt       = tilt
	msg.autofocus  = True

	# publish msg
	axis_pub.publish(msg)

	# wait until camera is at desired location before returning 
	desired_axis_state = {}
	desired_axis_state['pan']        = pan
	desired_axis_state['tilt']       = tilt
	desired_axis_state['autofocus']  = msg.autofocus
	wait_until_camera_at_desired_state(desired_axis_state)

#########################################################
# spin forever until the camera is at its desired state
#########################################################

######################################################################################
# get distance between two points
######################################################################################
def dist(point1, point2):
	x_dist = point1[0] - point2[0]
	y_dist = point1[1] - point2[1]
 	return math.sqrt(x_dist**2 + y_dist**2)
 	

#########################################################
# get a location nearby the robot
# this gets a point nearby to start_loc, this is parallel 
# to the init_angle and dist_away meters.
#########################################################
def get_nearby_loc(start_loc, init_angle, dist_away):
	y = dist_away * math.sin(init_angle)
	x = dist_away * math.cos(init_angle)
	nearby_loc_x = start_loc[0] + y # y is intentional.  It is parallel 
	nearby_loc_y = start_loc[1] - x # x is intentional.  IT is parallel
	return [nearby_loc_x, nearby_loc_y]

#########################################################
# get radian angle relative to start_angle and delta
#########################################################
def get_rad_angles(start_radians, delta_radians):
	start_angle = (start_radians - delta_radians) % (2*math.pi)
	end_angle   = (start_radians + delta_radians) % (2*math.pi)
	return [start_angle, end_angle]

#########################################################
# get_forward_goal
#########################################################
def get_forward_goal(start_loc, forward_angle, num_fwd_updates=0):
	if num_fwd_updates == 0:
		delta_angles = [0.262, 0.524, 0.785] # 15, 30, 45 degrees
	elif num_fwd_updates == 1:
		delta_angles = [0.262, 0.524] # 15, 30 degrees
	else:
		delta_angles = [0.262] # 15 degrees

	# incrementally get a wider and wider forward window
	for delta_angle in delta_angles: 
		try:
			# get the radian angle ranges 
			start_angle, end_angle = get_rad_angles(forward_angle, delta_angle)
			single_route_srv = rospy.ServiceProxy('/intersection_mapper/single_direction', singleRoute)
			data = single_route_srv(start_loc[0], start_loc[1], start_angle, end_angle)
		except rospy.ServiceException, e:
			log_message("\t\tService call failed: %s"%e)
			return [None, None]

		# if data found, break
		if len(data.x_world) > 0:
			break
	
	# if no data, return failure
	if len(data.x_world) == 0:
		return [None, None]

	# get goal
	goal = [data.x_world[-1], data.y_world[-1], forward_angle]
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, forward_angle)
	return [goal, quaternion]


#########################################################
# update_forward_goal
#########################################################
def update_forward_goal(robot_loc, forward_angle,num_fwd_updates):
	trans, _, _ = robot_loc
	# in the event that this is the first forward goal, the person may have materialized as an obstacle in 
	# front of us...or we may have turned with the wall slightly in front of us.  We attempt to find a path forward that starts nearby
	if num_fwd_updates == 0:
		furthest_goal       = None
		furthest_quaternion = None
		furthest_goal_dist  = 0.0 
		for horiz_delta in [0.0, -0.5, 0.5, -1.0, 1.0, -1.5, 1.5]:
			# compute nearby_loc
			nearby_loc = get_nearby_loc(trans, forward_angle, horiz_delta)
			new_goal, new_quaternion = get_forward_goal(nearby_loc, forward_angle, 0)
			if not new_goal == None: 
				goal_dist = dist(nearby_loc, new_goal)
				if goal_dist > furthest_goal_dist:
					furthest_goal_dist  = goal_dist
					furthest_goal       = new_goal
					furthest_quaternion = new_quaternion
		new_goal, new_quaternion = furthest_goal, furthest_quaternion
	# update our forward goal as normal
	else:
		new_goal, new_quaternion = get_forward_goal(trans, forward_angle, num_fwd_updates)
		#cur_goal, num_fwd_updates = set_forward_goal(cur_goal, new_goal, new_quaternion, num_fwd_updates)
	# return our current goal and how many times we have updated
	return new_goal, new_quaternion

#########################################################
# wait_until_camera_at_desired_state
#########################################################
def wait_until_camera_at_desired_state(desired_axis_state):
	global axis_state, fresh_image

	# look forever until camera state is at desired location
	while True:
		# if we got a new camera state update, check to see if it is what we desire
		#print 'desired state:', desired_axis_state
		#print 'axis    state:', axis_state
	
		if 'pan' in desired_axis_state and abs(desired_axis_state['pan'] - axis_state['pan']) > 1.0:
			continue
		if 'tilt' in desired_axis_state and abs(desired_axis_state['tilt'] - axis_state['tilt']) > 1.0:
			continue
		if 'zoom' in desired_axis_state and abs(desired_axis_state['zoom'] - axis_state['zoom']) > 10.0:
			continue
		if 'focus' in desired_axis_state and abs(desired_axis_state['focus'] - axis_state['focus']) > 1.0:
			continue
		if 'autofocus' in desired_axis_state and not desired_axis_state['autofocus'] == axis_state['autofocus']:
			continue
		# if you've gotten this far, everything is where you want it!
		break

		time.sleep(0.1)

	#print '\tcamera at desired state.'
	#print '\tcamera at desired state.  Pan:', axis_state['pan'],   '  Tilt:', axis_state['tilt'], \
	#						  '  Zoom:', axis_state['zoom'], '  Focus:', axis_state['focus']
			
	# now wait for a fresh image (so we don't get the last one that may have motion blur)
	time.sleep(0.35)
	fresh_image = False
	while not fresh_image:
		time.sleep(0.1)

#########################################################
# detect_text
#########################################################
def detect_text(frame):
	# create list of all words seen
	# text_locs[idx] = [word, x1, y1, x2, y2]
	text_locs = []
	client = vision.ImageAnnotatorClient()
	image  = vision.types.Image(content=cv2.imencode('.jpg', frame)[1].tostring())
	
	try:
		response = client.text_detection(image=image)
	except:
		return []

	for idx in range(1,len(response.text_annotations)):
		text_annotation = response.text_annotations[idx]
		word = unicodedata.normalize('NFKD', text_annotation.description).encode('ascii','ignore')
		x1 = min(text_annotation.bounding_poly.vertices[0].x, text_annotation.bounding_poly.vertices[1].x, \
				 text_annotation.bounding_poly.vertices[2].x, text_annotation.bounding_poly.vertices[3].x)
		x2 = max(text_annotation.bounding_poly.vertices[0].x, text_annotation.bounding_poly.vertices[1].x, \
				 text_annotation.bounding_poly.vertices[2].x, text_annotation.bounding_poly.vertices[3].x)
		y1 = min(text_annotation.bounding_poly.vertices[0].y, text_annotation.bounding_poly.vertices[1].y, \
				 text_annotation.bounding_poly.vertices[2].y, text_annotation.bounding_poly.vertices[3].y)
		y2 = max(text_annotation.bounding_poly.vertices[0].y, text_annotation.bounding_poly.vertices[1].y, \
				 text_annotation.bounding_poly.vertices[2].y, text_annotation.bounding_poly.vertices[3].y)
		text_locs.append([word.lower(), x1, y1, x2, y2])

	return text_locs

#########################################################
# get_doortag
#########################################################
def get_doortag(text_locs):
    # look for certain keywords
    doortags = []
    for text_loc in text_locs:
        # extract info into more useful names
        word, x1, y1, x2, y2 = text_loc
        tmp = "  " + word + "  "
        door_regex_str= "\s[^0-9][0-6][0-9]{1,2}[a-zA-Z]{0,1}[^0-9]\s"
        room_regex = re.compile(door_regex_str)
        regex_found = room_regex.findall(tmp)
        if len(regex_found) > 0:
            if tmp == regex_found[0]:
                doortags.append(word)
    return doortags

#########################################################
# check_expectation
#########################################################
def check_expectation(read_tag, expectation):
    alphabet = 'abcdefghijklmnopqrstuvwxyz'
    read_number, read_letter = None, None
    parity, value, less_than = False, False, False
    if read_tag[-1] in alphabet:
        read_number = int(read_tag[0:-1])
        read_letter = read_tag[-1].lower()
    #else just save the number
    else:
        read_number = int(read_tag)
    if read_number == expectation["value"][0] and read_letter == expectation["value"][1]:
        value = True
        parity = True
    elif read_number%2 == expectation["parity"]:
        parity = True
    if read_number < expectation["value"][0] or (expectation["value"][1] is not None and alphabet.index(read_letter) < alphabet.index(expectation["value"][1])):
        less_than = True
    return value, parity, less_than, read_number, read_letter

#########################################################
# doortag_reasoning
#########################################################
def doortag_reasoning(door_info, doortags, read_direction):
    alphabet = 'abcdefghijklmnopqrstuvwxyz'
    [door_label, door_number, door_direction] = door_info
    new_number, new_direction = None, None
    goal_found = False
    value, parity, less_than = False, False, False
    goal_number, goal_letter = None, None
    #if the goal has a letter at the end, save it off as a separate variable
    if door_label[-1].lower() in alphabet:
        goal_number = int(door_label[0:-1])
        goal_letter = door_label[-1].lower()
    #otherwise just save the number
    else:
        goal_number = int(door_label)
    read_number, read_letter = None, None
    expectation = {"value":[goal_number, goal_letter], "parity":goal_number%2}
    #if nothing was read, just check the next door
    if len(doortags) < 1:
        new_number = door_number+1
        new_direction = door_direction
    #if text was found, search it for possible door tags
    else:
        #loop over scraped text from this door
        for read_tag in doortags:
			
            if not goal_found:
				#if read tag has letter at the end, save it off
                value, parity, less_than, read_number, read_letter = check_expectation(read_tag, expectation)
				#if there's a match, goal was found
                if value:
                    goal_found = True
				#if the letters matched, use the difference to find index of door
                elif read_number == goal_number:
                    new_direction = read_direction
                    new_number = abs(alphabet.index(goal_letter) - alphabet.index(read_letter)) + door_number
                    
				#if nothing matches, apply common sense rules
                else:
					#if parity matches, compute index
                    if parity:
                        new_direction = read_direction
                        new_number = abs(goal_number - read_number)/2 + door_number
					#if parity doesn't match, start checking other side
                    else:
                        new_direction = 'left' if read_direction == 'right' else 'right'
                        new_number = 0
    return goal_found, new_number, new_direction, value, parity, less_than, read_number, read_letter

#########################################################
# check_missed
#########################################################
def check_missed(doors_checked, direction, door_label):
    missed = False
    alphabet = 'abcdefghijklmnopqrstuvwxyz'
    numbers = []
    letters = []
    order = ''
    goal_number, goal_letter = None, None
    #if the goal has a letter at the end, save it off as a separate variable
    if door_label[-1].lower() in alphabet:
        goal_number = int(door_label[0:-1])
        goal_letter = door_label[-1].lower()
    #otherwise just save the number
    else:
        goal_number = int(door_label)
    for thing in doors_checked[direction]:
        number, letter = thing[2][0], thing[2][1]
        
        numbers.append(number)
        letters.append(letter)
    if goal_number not in numbers:
        tmp = [x for x in numbers if x is not None]
        if len(tmp) > 1:
            missed = goal_number in range(min(tmp), max(tmp))
            if tmp[-1] > tmp[0]:
                if goal_number < tmp[0]:
                    missed = True
            elif tmp[-1] < tmp[0]:
                if goal_number > tmp[0]:
                    missed = True
                
    else:
        letter_idxs = []
        for i in range(len(letters)):
            if goal_number == numbers[i] and letters[i] is not None:
                these_letters.append(alphabet.index(letters[i]))
        if len(letter_idxs) > 0:
            missed = alphabet.index(goal_letter) in range(min(letter_idxs), max(letter_idxs)) 
    return missed
         
#########################################################
# setup a signal handler to listen for exit signal (ctrl + c)
#########################################################
def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

#########################################################
# movebase_client
#########################################################
def movebase_client(new_goal, wait_for_goal):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = new_goal[0]
    goal.target_pose.pose.position.y = new_goal[1]
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = new_goal[2][2]
    goal.target_pose.pose.orientation.w = new_goal[2][3]
    client.send_goal(goal)
    if wait_for_goal:
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()

#########################################################
# movebase_stop
#########################################################
def movebase_stop():
    log_message("Stopping!")
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.cancel_all_goals()

#########################################################
# get_doors
#########################################################
def get_doors(img, velodyne_data):
    global roihist, rotationMatrix
    #histogram backprojection to get doors
    imHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    dst = cv2.calcBackProject([imHSV],[0,1],roihist,[0,180,0,256],1)
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    cv2.filter2D(dst,-1,disc,dst)
    ret,maskHSV = cv2.threshold(dst,50,255,0)
    connectivity = 8 
    output = cv2.connectedComponentsWithStats(maskHSV, connectivity, cv2.CV_32S)
    boxes = output[2][:,[cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT]]

    #get robot position for base to map transformation
    [trans_vel_o, rot_vel, quaternion_vel]  = get_robot_position()
    #log_message(trans_vel_o)
    trans_vel = tuple(trans_vel_o) + ( 1,  )
    im = img.copy()
    rotationMatrix_vel = tf.transformations.quaternion_matrix( quaternion_vel )
    rotationMatrix_vel[ :, 3 ] = trans_vel

    #pointcloud calibration to get door locs
    vel_points = np.array(velodyne_data)
    #vel_points = vel_points[(vel_points[:,0] >= 3.0)]
    rings = vel_points[:,4]
    one_col = np.ones((vel_points.shape[0], 1))
    vel_points = np.hstack((vel_points[:,0:3], one_col))
    rotated_vel_points = rotationMatrix.dot(np.transpose(vel_points))
    uv_points = np.transpose(camera_model.P.dot(rotated_vel_points))
    uv_points = np.array(uv_points[:,0:2]/uv_points[:,2])
    map_vel_points = np.transpose(rotationMatrix_vel.dot(np.transpose(vel_points)))
    min_dist_cond = (vel_points[:,0] >= 0.0)
    uv_points = uv_points[min_dist_cond]
    vel_points = vel_points[min_dist_cond]
    rings = rings[min_dist_cond]
    map_vel_points = map_vel_points[min_dist_cond]

    ring_uv_points = uv_points[rings==8]
    ring_map_vel_points = map_vel_points[rings==8]
    doors_list = []
    doors_boxes = []

    #filter based on height and width
    for box in boxes:
        if box[2] > 1900 and box[3] > 1000:
            continue
        x1, y1, x2, y2 = box[0], box[1], box[0]+box[2], box[1]+box[3]
        inside_cond = np.logical_and(np.logical_and(uv_points[:,0] > x1, uv_points[:,0] < x2), np.logical_and(uv_points[:,1] > y1, uv_points[:,1] < y2))
        inside_vel_points = vel_points[inside_cond]
        if inside_vel_points.shape[0] > 10:
            min_3d = np.min(inside_vel_points, axis=0)
            max_3d = np.max(inside_vel_points, axis=0)
            height = max_3d[2] - min_3d[2]  
            left_side = np.argmin(abs(ring_uv_points[:,0] - x1))
            right_side = np.argmin(abs(ring_uv_points[:,0] - x2))
            map_left = ring_map_vel_points[left_side,:]
            map_right = ring_map_vel_points[right_side,:]
            if np.sqrt((map_left[0] - map_right[0])**2 + (map_left[1] - map_right[1])**2) < 2.0 and np.sqrt((map_left[0] - map_right[0])**2 + (map_left[1] - map_right[1])**2) > .5 and height > .5:
            #doors_list.append((map_left[0:2] + map_right[0:2])/2.0)
                doors_list.append([map_left[0:2], map_right[0:2]])
                doors_boxes.append([x1, y1, x2, y2])

    return doors_list, doors_boxes, [trans_vel_o, rot_vel, quaternion_vel]

#########################################################
# get_robot_position
#########################################################
def get_robot_position():
    # get robot's location
    global tf_
    now = rospy.Time.now()
    tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(1.0))
    (trans, quaternion) = tf_.lookupTransform( '/map', '/base_link', now )
    rot = tf.transformations.euler_from_quaternion(quaternion)
    return [trans, rot, quaternion]


#########################################################
# door_to_goal
#########################################################
def door_to_goal(idx, door_dict, direction):
    goal_type = None
    dist_factor = 1.0
    tmp = door_dict[direction+"_sides"][idx]
    if tmp in door_dict['left_sides']:
        goal_type = 'left'
    else:
        goal_type = 'right'

    side1 = tmp[0]
    side2 = tmp[1]
    side1_dist = np.sqrt((side1[0] - initial_loc[0][0])**2 + (side1[1] - initial_loc[0][1])**2)
    side2_dist = np.sqrt((side2[0] - initial_loc[0][0])**2 + (side2[1] - initial_loc[0][1])**2)
    if side1_dist < side2_dist:
        close_side, far_side = side1, side2
    else:
        close_side, far_side = side2, side1
    door_theta = math.atan2(far_side[1] - close_side[1], far_side[0] - close_side[0])
    door_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, door_theta)
    if goal_type == 'left':
        goal_close = [close_side[0] + dist_factor*math.cos(door_theta-math.pi/2), close_side[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
        goal_far = [far_side[0] + dist_factor*math.cos(door_theta-math.pi/2), far_side[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
        goal_center = [(goal_close[0]+goal_far[0])/2, (goal_close[1]+goal_far[1])/2, door_quaternion]
    else:
        goal_close = [close_side[0] + dist_factor*math.cos(door_theta+math.pi/2), close_side[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
        goal_far = [far_side[0] + dist_factor*math.cos(door_theta+math.pi/2), far_side[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
        goal_center = [(goal_close[0]+goal_far[0])/2, (goal_close[1]+goal_far[1])/2, door_quaternion]
    goal_dist = np.sqrt((robot_loc[0][0] - goal_center[0])**2 + (robot_loc[0][1] - goal_center[1])**2)
    return goal_center, goal_type


#########################################################
# make_door_dict
#########################################################
def make_door_dict(master_door_info, robot_loc, initial_loc):
    door_dict = {'all':np.array([]), 'left':np.array([]), 'right':np.array([]), 'left_sides':np.array([]), 'right_sides':np.array([]), 'all_sides':np.array([])}
    master_door_list_local, master_door_sides_local = master_door_info[:]
    if master_door_list_local.shape[0] > 1:
        path_theta = np.arctan2(robot_loc[0][1] - initial_loc[0][1], robot_loc[0][0] - initial_loc[0][0])
        door_thetas = np.arctan2(master_door_list_local[:,1] - initial_loc[0][1], master_door_list_local[:,0] - initial_loc[0][0])
        left_doors = master_door_list_local[door_thetas > path_theta]
        left_sides = master_door_sides_local[door_thetas > path_theta]
        right_doors = master_door_list_local[door_thetas < path_theta]
        right_sides = master_door_sides_local[door_thetas < path_theta]
        left_doors_dist = np.sqrt((left_doors[:,0] - initial_loc[0][0])**2 + (left_doors[:,1] - initial_loc[0][1])**2)
        right_doors_dist = np.sqrt((right_doors[:,0] - initial_loc[0][0])**2 + (right_doors[:,1] - initial_loc[0][1])**2)
        left_doors_idxs = np.argsort(left_doors_dist)
        right_doors_idxs = np.argsort(right_doors_dist)
        all_doors_idxs = np.argsort(master_door_list_local)
        door_dict['all'] = master_door_list_local[all_doors_idxs]
        door_dict['all_sides'] = master_door_sides_local[all_doors_idxs]
        door_dict['left'] = left_doors[left_doors_idxs]
        door_dict['right'] = right_doors[right_doors_idxs]
        door_dict['left_sides'] = left_sides[left_doors_idxs]
        door_dict['right_sides'] = right_sides[right_doors_idxs]
    return door_dict	

#########################################################
# navigate_loop
#########################################################
def navigate_loop():
    global master_door_info, forward, door_info, initial_loc, robot_loc, run, image_global, doors_checked, frozen, num_forward_updates, forward_missing, monitor_if_stuck
    log_message("started")
    missed = False
    exhaustive = False
    prev_goal = None
    goal = []
    goal_type = 'forward'
    dist_factor = 1.0
    goal_thresh = 1.0
    num_forward_updates = 0
    forward_missing = 0
    while True:
        #door goal is computed from quaternion transformation from the door theta
        #TODO classify and sort doors
        if run:
            monitor_if_stuck = True
            status.publish("running")
            # Check if stuck
            if robot_is_stuck:
                log_message('robot is stuck!')
                movebase_stop()
                status.publish("failure")
                time.sleep(0.1)
                continue

            #grab forward direction
            robot_loc = get_robot_position()
            travel_distance = np.sqrt((robot_loc[0][0] - initial_loc[0][0])**2 + (robot_loc[0][1] -initial_loc[0][1])**2)
            if travel_distance > 2.0:
                forward_angle = np.arctan2(robot_loc[0][1] - initial_loc[0][1], robot_loc[0][0] - initial_loc[0][0])
            else:
                _, _, forward_angle = tf.transformations.euler_from_quaternion(robot_loc[2])
            this_forward = update_forward_goal(robot_loc, forward_angle, num_forward_updates)[0]
            #generate dictionary of door localizations
            door_dict = make_door_dict(master_door_info, robot_loc, initial_loc)

            #get information to determine next door to drive to
            door_label, door_number, door_direction = door_info
            #print ("current goal info: ", door_info)
            #get doors for the current side of the hallway
            tmp_list = door_dict[door_direction+'_sides']

            #get list of all doors that have already been checked
            tmp_checked = np.array([x[0:2] for x in doors_checked[door_direction]])
            tmp_checked_readtags = [x[2] for x in doors_checked[door_direction] if x[2] is not None]
            #if missed:
                #print (door_info)
            
            if len(tmp_list) == tmp_checked.shape[0] and len(tmp_list) == len(tmp_checked_readtags) and this_forward is None and travel_distance > 1.0 and exhaustive:
                log_message("door not found, failure, stopping")
                run = False
                status.publish("failure")
            elif travel_distance > 1.0 and this_forward is None:
                if not exhaustive:
                    log_message("got to the end of the hallway, going back")
                    frozen = True
                    movebase_client([initial_loc[0][0], initial_loc[0][1], initial_loc[2]], True)
                    frozen = False
                    door_info = [door_label, 0, door_direction]
                    exhaustive = True
                    prev_goal = None
                    num_forward_updates = 0
                else:
                    log_message("door not found")
                    run = False
                    status.publish("failure")
                    continue
            elif this_forward is None:
                forward_missing+=1
                if forward_missing > 100:
                    log_message("forward missing for too long, failure")
                    run = False
                    status.publish("failure")
                    continue
            else:
                forward_missing = 0
            #if there are doors and the one we want is in the list, get it's location
            if door_dict[door_direction].shape[0] > 0 and door_number < len(tmp_list):
                #log_message("doors exist, get one")
                goal_center, goal_type = door_to_goal(door_number, door_dict, door_direction)
                
                if tmp_checked.shape[0] > 0:
                    distances = np.sqrt((tmp_checked[:,0] - goal_center[0])**2 + (tmp_checked[:,1] - goal_center[1])**2)
                    min_dist_idx = np.argmin(distances)
                    #if door has already been checked, just go to the next one
                    if distances[min_dist_idx] < 0.5 and doors_checked[door_direction][min_dist_idx][2][0] is not None:
                        log_message("door has already been checked")
                        door_info = [door_label, door_number+1, door_direction]
                        prev_goal = None
                        print (door_info)
                        continue

                # compute distance to goal
                goal_dist = np.sqrt((robot_loc[0][0] - goal_center[0])**2 + (robot_loc[0][1] - goal_center[1])**2)
                
                #if the goal is close, just drive to it directly
                if goal_dist < 2.0:
                    log_message("goal is close, drive up")
                    goal = goal_center
                    print (goal)
                    log_message("Driving to door")
                    # drive up to door
                    movebase_client(goal, True)
                    image1, image2 = None, None
                    # pan camera accordingly
                    if goal_type == 'left':
                        set_pt(-5.0, 30.0)
                        image1 = image_global
                        set_pt(15.0, 30.0)
                        image2 = image_global
                    else:
                        set_pt(-175.0, 30.0)
                        image1 = image_global
                        set_pt(165.0, 30.0)
                        image2 = image_global

                    # read door tags
                    text_locs = detect_text(np.vstack((image1, image2)))

                    # set camera back to home
                    set_pt(90.0, 0.0)
                    print ('text',get_doortag(text_locs))

                    #ascertain if goal is found and do some common sense reasoning
                    goal_found, goal_num, goal_dir, value, parity, less_than, read_number, read_letter = doortag_reasoning(door_info, get_doortag(text_locs), goal_type)
                    print ('reasoning',goal_found, goal_dir, goal_num)

                    #if goal is found, we are done  
                    if goal_found:
                        status.publish("complete")
                        run = False
                        continue

                    #update list of checked doors
                    
                    doors_checked[door_direction].append([goal[0], goal[1], [read_number, read_letter]])

                    #if we haven't exceeded the number of checked doors for common sense, use the common sense predicted direction and number
                    if len(doors_checked[door_direction]) < 2:
                        log_message("setting next door as goal")
                        door_info = [door_label, goal_num, goal_dir]
                        

                    #if we have, check if the door was missed
                    else:
                        #log_message("check if door was missed")
                        door_info = [door_label, door_number+1, goal_dir]
                        if not exhaustive:
                            missed = check_missed(doors_checked, goal_dir, door_info[0])
                            if missed:
                                log_message("door was missed, go back")
                                frozen = True
                                movebase_client([initial_loc[0][0], initial_loc[0][1], initial_loc[2]], True)
                                frozen = False
                                door_info = [door_label, 0, goal_dir]
                                exhaustive = True
                                prev_goal = None
                    prev_goal = None
                    print ("current goal info: ", door_info)
                
                #otherwise keep driving straight
                else:
                    goal_type = 'forward'
                    if this_forward is not None:
                        forward_theta = math.atan2(this_forward[1] - robot_loc[0][1], this_forward[0] - robot_loc[0][0])
                        forward_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, forward_theta)
                        goal = [this_forward[0], this_forward[1], forward_quaternion]

                        goal_dist = 0
                        if prev_goal is not None:
                            goal_dist = np.sqrt((robot_loc[0][0] - prev_goal[0])**2 + (robot_loc[0][1] - prev_goal[1])**2)
                            #print ("goal dist: ", goal_dist)
                        if goal_dist < goal_thresh:
                            log_message("driving forward")
                            movebase_client(goal, False)
                            num_forward_updates+=1
                            prev_goal = goal
                    else:
                        forward_missing+=1
                        if forward_missing > 100:
                            log_message("forward missing for a while, failure")
                            run = False
                            status.publish("failure")
                            continue
            else:
                log_message("no doors to check, just drive forward")
                goal_type = 'forward'
                if this_forward is not None:
                    forward_missing = 0
                    forward_theta = math.atan2(this_forward[1] - robot_loc[0][1], this_forward[0] - robot_loc[0][0])
                    forward_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, forward_theta)
                    goal = [this_forward[0], this_forward[1], forward_quaternion]

                    goal_dist = 0
                    if prev_goal is not None:
                        goal_dist = np.sqrt((robot_loc[0][0] - prev_goal[0])**2 + (robot_loc[0][1] - prev_goal[1])**2)
                        #print ("goal dist: ", goal_dist)
                    if goal_dist < goal_thresh:
                        log_message("driving forward")
                        movebase_client(goal, False)
                        num_forward_updates+=1
                        prev_goal = goal
                else:
                    forward_missing+=1
                    if forward_missing > 100:
                        log_message("forward missing for a while, failure")
                        run = False
                        status.publish("failure")
                        continue

        else:
            #status.publish("idle")
            monitor_if_stuck = False
            time.sleep(1)

#########################################################
# main callback
#########################################################
def callback(image, camera_info, velodyne, routes):
    global data_chunk, all_doors, master_door_info, forward, robot_loc, fresh_image, image_global, frozen
    fresh_image = True
    data_chunk = {}
    camera_model.fromCameraInfo(camera_info)
    data_chunk['camera'] = camera_model
    assert isinstance(velodyne, PointCloud2)
    gen = pc2.read_points(velodyne)
    # collect the points
    points = []
    for p in gen:
        points.append(p)
    data_chunk['velodyne_points'] = points
    img = bridge.imgmsg_to_cv2(image, "bgr8")
    image_global = img
    data_chunk['image'] = img
    data_chunk['map'] = []
    door_detections, doors_boxes, robot_loc = get_doors(img, points)
    #for box in doors_boxes:
    #    cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
    #cv2.imshow('dst', img)
    #cv2.waitKey(0)
    #sys.exit()
    master_door_list = np.array([])
    master_door_sides_list = np.array([])
    if not frozen:
        all_doors.extend(door_detections)
    if len(all_doors) > 1:
        #print (all_doors)
        door_locs = np.array(all_doors)
        door_locs1 = door_locs[:,0,0:2]
        door_locs2 = door_locs[:,1,0:2]
        door_locs = (door_locs1+door_locs2)/2.0
        door_clusters =  fclusterdata(door_locs, .5, criterion='distance') #input, cluster_thresh, criterion
        tmp_door_list = []
        tmp_door_sides_list = []
        for j in range(1, max(door_clusters)+1):
            if sum(door_clusters==j) > 0:
                door_cluster = door_locs[door_clusters==j,:]
                door_cluster1 = door_locs1[door_clusters==j,:]
                door_cluster2 = door_locs2[door_clusters==j,:]

                door_centroid = np.mean(door_cluster, axis=0)
                door_centroid1 = np.mean(door_cluster1, axis=0)
                door_centroid2 = np.mean(door_cluster2, axis=0)
                tmp_door_list.append(door_centroid)
                tmp_door_sides_list.append([door_centroid1, door_centroid2])

        master_door_list = np.array(tmp_door_list)
        master_door_sides_list = np.array(tmp_door_sides_list)
    master_door_info = [master_door_list, master_door_sides_list]
    routes_dict = json.loads(routes.data)#['dirs']['forward'][3.6]
    if 'forward' in routes_dict['dirs'].keys():
        forward = []
        furthest_dist = 0
        for key in routes_dict['dirs']['forward'].keys():
            if key > furthest_dist:
                forward = routes_dict['dirs']['forward'][key]
                furthest_dist = key
    else:
        forward = None
    #print (forward)

#########################################################
# service_request
#########################################################
def request(data):
    global door_label, doors_checked, door_number, initial_quaternion, door_direction, run, initial_loc, robot_loc, door_info, num_forward_updates, forward_missing, frozen
    log_message("Got new command: " + data.operation)
    if data.operation == 'start':
        master_door_list, master_door_sides_list, all_doors = [], [], []
        door_label = str(data.door_tag)
        door_number = data.door_index
        door_direction = data.door_dir
        doors_checked = {'left':[], 'right':[], 'all':[]}
        door_info = [door_label, door_number, door_direction]
        num_forward_updates = 0
        forward_missing = 0
        print ("STARTING:", door_label, door_number, door_direction)
        initial_loc = get_robot_position()
        robot_loc = initial_loc
        run = True
        frozen = False
        return doorTagResponse(True)
    if data.operation == 'stop':
        movebase_stop()
        run = False
        frozen = True
        return doorTagResponse(False)

#########################################################
# check if robot is stuck
# prev_pos_time = [trans, rad_angle, time]
#########################################################
def robot_stuck(prev_pos_time, cur_pos_time):
	# refer to global variables
	global robot_stuck_timeout, robot_is_stuck
	
	# if < robot_stuck_timeout seconds has elapsed, the robot is not stuck
	if (cur_pos_time[2] - prev_pos_time[2]) < robot_stuck_timeout:
		robot_is_stuck = False
		return prev_pos_time

	# if the robot has moved > 0.1 meters over that time interval, the robot is not stuck
	if dist(prev_pos_time[0], cur_pos_time[0]) > 0.1:
		robot_is_stuck = False
		return cur_pos_time

	# if the robot has rotated > 10 degrees over that time interval, the robot is not stuck
	rad_rotated = (prev_pos_time[1] - cur_pos_time[1]) % (2*math.pi)
	if abs(rad_rotated) > 0.175:
		robot_is_stuck = False
		return cur_pos_time

	# at this point, the robot is definitely stuck!
	robot_is_stuck = True
	return prev_pos_time

#########################################################
# monitor if robot gets stuck
#########################################################
def monitor_robot_stuck(data):
	# refer to global variables
	global monitor_if_stuck

	# initialize robot position and time
	cur_time = time.time()
	[trans, rot, _] = get_robot_position()
	prev_pos_time = [trans, rot[2], cur_time]

	while True:
		if not monitor_if_stuck:
			# reset robot position and time
			cur_time = time.time()
			[trans, rot, _] = get_robot_position()
			prev_pos_time = [trans, rot[2], cur_time]
		else:
			# get the current time and position of robot 
			cur_time = time.time()
			[trans, rot, _] = get_robot_position()
			cur_pos_time = [trans, rot[2], cur_time]
			# check if the robot is stuck
			prev_pos_time = robot_stuck(prev_pos_time, cur_pos_time)

		time.sleep(0.5)

#########################################################
# main code
#########################################################
if __name__=="__main__":
    global tf_

    # open output log
    dateTimeObj        = datetime.now()
    output_filename    = dateTimeObj.strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
    home               = os.path.expanduser("~")
    log_path           = home + '/.ros/door_navigation/log/'
    if not os.path.exists(log_path):
        os.makedirs(log_path)
    output_log         = open(log_path + output_filename, 'w')
    log_message('Starting door navigate')
  
    rospy.init_node('door_navigation')
    tf_ = tf.TransformListener()
    image_sub = message_filters.Subscriber('/axis/image_raw_out', Image)
    cam_info_sub = message_filters.Subscriber('/axis/camera_info', CameraInfo)
    velodyne_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)
    routes_sub   = message_filters.Subscriber("/intersection_mapper/routes", Routes2)
    axis_sub     = rospy.Subscriber('/axis/state', Axis, update_state)
    axis_pub     = rospy.Publisher('/axis/cmd', Axis, queue_size=1)
    s            = rospy.Service('/door_navigation/command', doorTag, request)
    status      = rospy.Publisher('/door_navigation/status', String, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, cam_info_sub, velodyne_sub, routes_sub], 10,.1)#, allow_headerless= True)
    ts.registerCallback(callback)

    # wait one second, then set flag to start monitoring for robot being stuck
    rospy.Subscriber('/door_navigation/start_monitor_stuck', Bool, monitor_robot_stuck)
    monitor_stuck_pub = rospy.Publisher('/door_navigation/start_monitor_stuck',  Bool, queue_size=1)
    time.sleep(1)
    monitor_stuck_pub.publish(True)

    #rospy.spin()
    navigate_loop()
