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

from scipy.spatial.distance import cdist, pdist
from scipy.cluster.hierarchy import fclusterdata
from scipy.optimize import linear_sum_assignment


import message_filters
import rospy
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
import actionlib
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
from intersection_mapper.msg import Routes2
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


all_doors = []
master_door_list = np.array([])
master_door_sides_list = np.array([])
master_door_info = [master_door_list, master_door_sides_list]
forward = None
run = False
axis_state = {}
new_update = False
fresh_image = False
image_global = None
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

def get_doortag(text_locs):
    # look for certain keywords
    doortags = []
    for text_loc in text_locs:
        # extract info into more useful names
        word, x1, y1, x2, y2 = text_loc

        # ignore one-letter words
        if len(word) < 2:
            continue
        regexes_data = "[1-3][0-9]{2}[a-zA-Z]{0,1}===[1-3][eE][0-9]{2}"
        regexes_string = regexes_data.split('===')
        for i in range(len(regexes_string)):
            p = re.compile(regexes_string[i])
            regexes_string[i] = p
        regexes = regexes_string
        this_word_is_regex = False
        for regex in regexes:
            regex_found = regex.findall(word)
            if len(regex_found) > 0:
                this_word_is_regex = True
        # we found a regex.  Due to the distance, the word may not perfectly accurate.
        # we will save it off now, but make note that it was found while passively scraping
        if this_word_is_regex:
            doortags.append(word)
    return doortags

def doortag_reasoning(door_info, doortags, read_direction):
    [door_label, door_number, door_direction] = door_info
    new_number, new_direction = None, None
    goal_found = False
    alphabet = 'abcdefghijklmnopqrstuvwxyz'
    goal_number, goal_letter = None, None
    if door_label[-1].lower() in alphabet:
        goal_number = int(door_label[0:-1])
        goal_letter = door_label[-1].lower()
    else:
        goal_number = int(door_label)
    read_number, read_letter = None, None
    if len(doortags) < 1:
        new_number = door_number+1
        new_direction = door_direction
    else:
        for read_tag in doortags:
            print (read_tag, read_tag[-1])
            if not goal_found:
                if read_tag[-1] in alphabet:
                    read_number = int(read_tag[0:-1])
                    read_letter = read_tag[-1].lower()
                else:
                    read_number = int(read_tag)
                if read_number == goal_number and read_letter == goal_letter:
                    goal_found = True
                elif read_number == goal_number:
                    new_direction = read_direction
                    new_number = abs(alphabet.index(goal_letter) - alphabet.index(read_letter))
                else:
                    if read_letter is None:
                        if read_number%2 == goal_number%2:
                            new_direction = read_direction
                            new_number = abs(goal_number - read_number)/2
                        else:
                            new_direction = 'left' if read_direction == 'right' else 'right'
                            new_number = 0
    return goal_found, new_number, new_direction
    
    
   
    #TODO implement commmon sense about doortags
#########################################################
# setup a signal handler to listen for exit signal (ctrl + c)
#########################################################
def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

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

def movebase_stop():
    print "Stopping!"
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.cancel_all_goals()

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
    #print(trans_vel_o)
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

def get_robot_position():
    # get robot's location
    global tf_
    now = rospy.Time.now()
    tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(0.1))
    (trans, quaternion) = tf_.lookupTransform( '/map', '/base_link', now )
    rot = tf.transformations.euler_from_quaternion(quaternion)
    return [trans, rot, quaternion]

def callback(image, camera_info, velodyne, routes):
    global data_chunk, all_doors, master_door_info, forward, robot_loc, fresh_image, image_global
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
    all_doors.extend(door_detections)
    if len(all_doors) > 1:
        #print (all_doors)
        door_locs = np.array(all_doors)
        door_locs1 = door_locs[:,0,0:2]
        door_locs2 = door_locs[:,1,0:2]
        door_locs = (door_locs1+door_locs2)/2.0
        door_clusters =  fclusterdata(door_locs, .25, criterion='distance') #input, cluster_thresh, criterion
        tmp_door_list = []
        tmp_door_sides_list = []
        for j in range(1, max(door_clusters)+1):
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
    
    #print (forward)

def navigate_loop():
    global master_door_info, forward, door_info, initial_loc, robot_loc, run, image_global
    print ("started")
    door_dict = {'all':[], 'left':[], 'right':[], 'left_sides':[], 'right_sides':[]}
    prev_goal = None
    goal = []
    goal_type = ''
    dist_factor = 1.0
    goal_thresh = 1.0
    while True:
        #door goal is computed from quaternion transformation from the door theta
        #TODO classify and sort doors
        if run:
            this_forward = forward 
            master_door_list_local, master_door_sides_local = master_door_info[:]
            path_theta = np.arctan2(robot_loc[0][1] - initial_loc[0][1], robot_loc[0][0] - initial_loc[0][0])
            door_thetas = np.arctan2(master_door_list_local[:,1] - initial_loc[0][1], master_door_list_local[:,0] - initial_loc[0][0])
            #TODO check this polarity
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
            #TODO choose navigation command
            door_label, door_number, door_direction = door_info
            tmp_list = door_dict[door_direction+'_sides']
            if door_dict['all'].shape[0] > 0 and door_number < len(tmp_list):
                
                tmp = tmp_list[door_number]
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
                if goal_dist < 2.0:
                    goal = goal_center
                else:
                    try:
                        forward_theta = math.atan2(this_forward[1] - robot_loc[0][1], this_forward[0] - robot_loc[0][0])
                        forward_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, forward_theta)
                        goal = [this_forward[0], this_forward[1], forward_quaternion]
                        goal_type = 'forward'
                    except:
                        print (robot_loc, this_forward)
                
            else:
                try:
                    forward_theta = math.atan2(this_forward[1] - robot_loc[0][1], this_forward[0] - robot_loc[0][0])
                    forward_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, forward_theta)
                    goal = [this_forward[0], this_forward[1], forward_quaternion]
                    goal_type = 'forward'
                except:
                    print (robot_loc, this_forward)

            #TODO perform navigation
            if goal_type == 'forward':
                #print ("Driving forward")
                if prev_goal is not None:
                    goal_dist = np.sqrt((robot_loc[0][0] - prev_goal[0])**2 + (robot_loc[0][1] - prev_goal[1])**2)
                    if goal_dist < goal_thresh:
                        print ("updating forward")
                        movebase_client(goal, False)
                        prev_goal = goal
                else:
                    print ("first forward")
                    movebase_client(goal,False)
                    prev_goal = goal
            
            else:
                print ("Driving to door")
                movebase_client(goal, True)
                if goal_type == 'left':
                    set_pt(0.0, 30.0)
                else:
                    set_pt(180.0, 30.0)
                text_locs = detect_text(image_global)
                set_pt(90.0, 0.0)
                print ('text',get_doortag(text_locs))
                goal_found, goal_dir, goal_num = doortag_reasoning(door_info, get_doortag(text_locs), goal_type)
                print ('reasoning',goal_found, goal_dir, goal_num)
                if goal_found:
                    sys.exit()
                else:
                    door_info = [door_label, goal_dir, goal_num]
                    prev_goal = None
                #TODO read door tag once navigation completes
                #TODO modify door_info as per read door tag
            
            
        else:
            time.sleep(1)
def request(data):
    global door_label, door_number, initial_quaternion, door_direction, run, initial_loc, door_info
    if data.operation == 'start':
        master_door_list, master_door_sides_list = [], []
        door_label = str(data.door_tag)
        door_number = data.door_index
        door_direction = data.door_dir
        door_info = [door_label, door_number, door_direction]
        print ("STARTING:", door_label, door_number, door_direction)
        initial_loc = get_robot_position()
        run = True
        return doorTagResponse(True)
    if data.operation == 'stop':
        movebase_stop()
        run = False
        return doorTagResponse(False)

if __name__=="__main__":
    global tf_
  
    rospy.init_node('door_navigation')
    tf_ = tf.TransformListener()
    image_sub = message_filters.Subscriber('/axis/image_raw_out', Image)
    cam_info_sub = message_filters.Subscriber('/axis/camera_info', CameraInfo)
    velodyne_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)
    routes_sub   = message_filters.Subscriber("/intersection_mapper/routes", Routes2)
    axis_sub     = rospy.Subscriber('/axis/state', Axis, update_state)
    axis_pub     = rospy.Publisher('/axis/cmd', Axis, queue_size=1)
    s            = rospy.Service('/door_navigation/command', doorTag, request)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, cam_info_sub, velodyne_sub, routes_sub], 10,.1)#, allow_headerless= True)
    ts.registerCallback(callback)
    #rospy.spin()
    navigate_loop()
