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
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from door_navigation.srv import *
from google.cloud import vision
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/robot-slang/husky-custom/src/rviz_marker_publisher/src/')
import rviz_marker_publisher

from generate_doors import *

bridge = CvBridge()
camera_model = PinholeCameraModel()
roihist = pickle.load(open('/home/tilyevsk/robot-slang/husky-custom/src/door_detection/src/roihist.pkl', 'rb'))

#set up darknet yolo
#import darknet
#darknet.set_gpu(1)
#net = darknet.load_net('/home/tilyevsk/robot-slang/husky-custom/src/door_navigation/cfg/yolo-obj.cfg', '/home/tilyevsk/robot-slang/husky-custom/src/door_navigation/cfg/yolo-obj.weights', 0)
#meta = darknet.load_meta('/home/tilyevsk/robot-slang/husky-custom/src/door_navigation/cfg/coco.data')

#net = darknet.load_net("/home/tilyevsk/darknet/cfg/yolov3-spp.cfg", "/home/tilyevsk/darknet/weights/yolov3-spp.weights", 0)
#meta = darknet.load_meta("/home/tilyevsk/darknet/cfg/coco.data")

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

all_doors, all_doors_sides = [], []
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
state = 'idle'
num_forward_updates = 0
forward_missing = 0
robot_is_stuck = False
monitor_if_stuck = False
robot_stuck_timeout = 16.0
map_data_global = {}
internal_status = 'standby'
last_internal_status = 'standby'
command = 'stop'
move_base_goal_status = -2
last_move_base_goal_status = None
new_goal_active = False

######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
    print(msg)
    output_log.write(datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + ' ' + msg + '\n')


#########################################################
# go to absolute PT position
# PARAM           RANGE                       DESCRIPTION
# pan             [-180,180] or [0.0, 1.0]    Pan value in degrees (or percent in absolute mode)
# tilt            [-90,90] or [0.0, 1.0]      Tilt value in degrees (or percent in absolute mode)
# brightness      [1,9999]                    Brightness
# focus           [1,9999]                    Focus
# autofocus       True/False                  Autofocus
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


######################################################################################
# Send the camera back to its home position
######################################################################################
def go_home():
    # refer to global object
    global axis_state

    # if already in home state, return; otherwise command camera to home state
    if axis_state['pan'] == 90.0 and axis_state['tilt'] == 0.0 and axis_state['zoom'] == 1.0:
        return
    else:
        set_pt(90.0, 0.0)

#########################################################
# wait_until_camera_at_desired_state
#########################################################
def wait_until_camera_at_desired_state(desired_axis_state):
    global axis_state, fresh_image

    # look forever until camera state is at desired location
    start_time = time.time()
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
    #                          '  Zoom:', axis_state['zoom'], '  Focus:', axis_state['focus']
            
    # now wait for a fresh image (so we don't get the last one that may have motion blur)
    time.sleep(0.35)
    fresh_image = False
    while not fresh_image:
        time.sleep(0.1)


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
# detect_text
#########################################################
def detect_text(frame):
    # create list of all words seen
    # text_locs[idx] = [word, x1, y1, x2, y2]
    text_locs = []
    image  = vision.types.Image(content=cv2.imencode('.jpg', frame)[1].tostring())
    
    try:
        ocr_client = vision.ImageAnnotatorClient()
        response = ocr_client.text_detection(image=image, timeout=5)
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

######################################################################################
# move_base_callback
# uint8 PENDING         = 0   # The goal has yet to be processed by the action server
# uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
# uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
#                             #   and has since completed its execution (Terminal State)
# uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
# uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
#                             #    to some failure (Terminal State)
# uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
#                             #    because the goal was unattainable or invalid (Terminal State)
# uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
#                             #    and has not yet completed execution
# uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
#                             #    but the action server has not yet confirmed that the goal is canceled
# uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
#                             #    and was successfully cancelled (Terminal State)
# uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
#                             #    sent over the wire by an action server
# Source: http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
######################################################################################
def move_base_callback(data):
    global move_base_goal_status, last_move_base_goal_status

    if len(data.status_list) == 0:         # no goals
        move_base_goal_status = -2         # indicates 'no goals'
    else:
        move_base_goal_status = data.status_list[-1].status

    if not last_move_base_goal_status == move_base_goal_status:
        last_move_base_goal_status = move_base_goal_status
        print 'move_base_goal_status:', move_base_goal_status

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
        door_regex_str= "\s[^0-9][a-zA-Z]{0,1}[0-9]{1,4}[a-zA-Z]{0,1}[^0-9]\s"
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
    new_number, new_direction = None, 'all'
    goal_found = False
    value, parity, less_than = False, False, False
    goal_number, goal_letter = None, None
    #if the goal has a letter at the end, save it off as a separate variable
    read_number, read_letter = None, None
    total_tag = ''
    tried_flag = False
    try:
        if door_label[-1].lower() in alphabet:
            goal_number = int(door_label[0:-1])
            goal_letter = door_label[-1].lower()
        #otherwise just save the number
        else:
            goal_number = int(door_label)
        
        expectation = {"value":[goal_number, goal_letter], "parity":goal_number%2}
        #if nothing was read, just check the next door
        if len(doortags) < 1:
            new_number = door_number+1
            new_direction = door_direction
        #if text was found, search it for possible door tags
        else:
            #loop over scraped text from this door
            for read_tag in doortags:
                total_tag = read_tag
                if not goal_found and read_tag[0] == door_label[0]:
                    #if read tag has letter at the end, save it off
                    tried_flag = True
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
            if not tried_flag:
                new_number = door_number+1
                new_direction = door_direction
                 
    except:
        for read_tag in doortags:
            total_tag = read_tag
            if not goal_found:
                if read_tag.lower() == door_label.lower():
                    goal_found = True
        new_number = door_number+1
        new_direction = 'all'
    return goal_found, new_number, new_direction, value, parity, less_than, read_number, read_letter, total_tag

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
    try:
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
                    letter_idxs.append(alphabet.index(letters[i]))
            if len(letter_idxs) > 0:
                missed = alphabet.index(goal_letter) in range(min(letter_idxs), max(letter_idxs)) 
    except:
        missed = False
    return missed
         
#########################################################
# setup a signal handler to listen for exit signal (ctrl + c)
#########################################################
def signal_handler(sig, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

######################################################################################
# keep track of when the goal is active
######################################################################################
def set_new_goal_active():
    # refer to global objects
    global new_goal_active
    new_goal_active = True

#########################################################
# movebase_client
#########################################################
def movebase_client(new_goal, wait_for_goal):
    # refer to global objects
    global new_goal_active, robot_is_stuck, move_base_goal_status

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

    # plot goal in rviz
    this_marker = {'id': 6400, 'x': new_goal[0], 'y': new_goal[1], 'quaternion': new_goal[2], 'color': [0.0, 1.0, 1.0], 'scale': [1.0, 0.2, 0.2]}
    goal_marker = rviz_marker_publisher.create_arrow_marker(this_marker)
    this_marker = {'id': 6401, 'x': new_goal[0], 'y': new_goal[1], 'name': 'goal', 'color': [0.0, 1.0, 0.0], 'scale': 0.4}
    text_marker = rviz_marker_publisher.create_text_marker(this_marker)
    rviz_marker_publisher.display_markers(rviz_publisher, [goal_marker, text_marker])

    # Sends the goal to the action server, waits for it to become active, then return control
    new_goal_active = False
    client.send_goal(goal, active_cb=set_new_goal_active)
    while not new_goal_active:
        time.sleep(0.1)
    # wait for the goal to move from 'nothing'/'pending' to something else (like 'active' or a terminal state)
    while move_base_goal_status in [-2, -1, 0]:
        time.sleep(0.1)

    if wait_for_goal:
        while move_base_goal_status < 2:
            if robot_is_stuck:
                log_message('robot is stuck!')
                log_message('no longer going to wait_for_goal!')
                return False
            time.sleep(0.1)
        return True

#########################################################
# movebase_stop
#########################################################
def movebase_stop():
    log_message("Stopping!")
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.cancel_all_goals()

    # wait for all goals to reach terminal state before returning control
    while move_base_goal_status in [-2, -1, 0, 1, 6, 7, 9]:
        time.sleep(0.1)

    # delete goal markers
    delete_marker1 = rviz_marker_publisher.create_delete_marker(6400)
    delete_marker2 = rviz_marker_publisher.create_delete_marker(6401)
    rviz_marker_publisher.display_markers(rviz_publisher, [delete_marker1, delete_marker2])

#########################################################
# get_doors
#########################################################
def get_doors(camera_model, img, velodyne_data, robot_loc):
    global roihist, rotationMatrix
    #histogram backprojection to get doors
    #imHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #dst = cv2.calcBackProject([imHSV],[0,1],roihist,[0,180,0,256],1)
    #disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    #cv2.filter2D(dst,-1,disc,dst)
    #ret,maskHSV = cv2.threshold(dst,50,255,0)
    #connectivity = 8 
    #output = cv2.connectedComponentsWithStats(maskHSV, connectivity, cv2.CV_32S)
    #boxes = output[2][:,[cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT]]
    boxes, scores, door_detections = get_doors_general(camera_model, img, velodyne_data, robot_loc)
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
    doors_median_list = []
    doors_boxes = []
    scores_list = []
    #filter based on height and width
    for i in range(len(boxes)):
        box = boxes[i]
        if box[2] > 1900 and box[3] > 1000 or scores[i] < 0.75:
            continue
        x1, y1, x2, y2 = box
        inside_cond = np.logical_and(np.logical_and(uv_points[:,0] > x1, uv_points[:,0] < x2), np.logical_and(uv_points[:,1] > y1, uv_points[:,1] < y2))
        inside_vel_points = vel_points[inside_cond]
        if inside_vel_points.shape[0] > 10:
            median_3d = np.median(map_vel_points[inside_cond], axis=0)
            min_3d = np.min(inside_vel_points, axis=0)
            max_3d = np.max(inside_vel_points, axis=0)
            height = max_3d[2] - min_3d[2]  
            left_side = np.argmin(abs(ring_uv_points[:,0] - x1))
            right_side = np.argmin(abs(ring_uv_points[:,0] - x2))
            map_left = ring_map_vel_points[left_side,:]
            map_right = ring_map_vel_points[right_side,:]
            if np.sqrt((map_left[0] - map_right[0])**2 + (map_left[1] - map_right[1])**2) < 2.0 and np.sqrt((map_left[0] - map_right[0])**2 + (map_left[1] - map_right[1])**2) > .5 and height > .5:
            #doors_list.append((map_left[0:2] + map_right[0:2])/2.0)
                doors_median_list.append(median_3d[0:2])
                doors_list.append([map_left[0:2], map_right[0:2]])
                doors_boxes.append([x1, y1, x2, y2])
                scores_list.append(scores[i])

    return doors_list, doors_boxes, [trans_vel_o, rot_vel, quaternion_vel], scores_list, doors_median_list



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
    global initial_loc
    goal_type = None
    goal_index = None
    dist_factor = 1.0
    tmp = door_dict[direction+"_sides"][idx]
    
    if tmp in door_dict['left_sides']:
        goal_type = 'left'
        goal_index = np.where(door_dict['left_sides'] == tmp)[0][0]
    else:
        goal_type = 'right'
        goal_index = np.where(door_dict['right_sides'] == tmp)[0][0]
    goal_median = door_dict[direction][idx]
    side1 = tmp[0]
    side2 = tmp[1]
    
    side1_dist = np.sqrt((side1[0] - initial_loc[0][0])**2 + (side1[1] - initial_loc[0][1])**2)
    side2_dist = np.sqrt((side2[0] - initial_loc[0][0])**2 + (side2[1] - initial_loc[0][1])**2)

    #if direction == 'all':
    #    print (tmp)
    #    print (side1)
    #    print (side2)
    #    print (side1_dist, side2_dist)
    #    print (goal_type)

    if side1_dist < side2_dist:
        close_side, far_side = side1, side2
    else:
        close_side, far_side = side2, side1
    door_theta = math.atan2(far_side[1] - close_side[1], far_side[0] - close_side[0])
    door_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, door_theta)
    if goal_type == 'left':
        goal_close = [close_side[0] + dist_factor*math.cos(door_theta-math.pi/2), close_side[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
        goal_far = [far_side[0] + dist_factor*math.cos(door_theta-math.pi/2), far_side[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
        #goal_center = [(goal_close[0]+goal_far[0])/2, (goal_close[1]+goal_far[1])/2, door_quaternion]
        goal_center = [goal_median[0] + dist_factor*math.cos(door_theta-math.pi/2), goal_median[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
    else:
        goal_close = [close_side[0] + dist_factor*math.cos(door_theta+math.pi/2), close_side[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
        goal_far = [far_side[0] + dist_factor*math.cos(door_theta+math.pi/2), far_side[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
        #goal_center = [(goal_close[0]+goal_far[0])/2, (goal_close[1]+goal_far[1])/2, door_quaternion]
        goal_center = [goal_median[0] + dist_factor*math.cos(door_theta+math.pi/2), goal_median[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
    goal_dist = np.sqrt((robot_loc[0][0] - goal_center[0])**2 + (robot_loc[0][1] - goal_center[1])**2)
    return goal_center, goal_close, goal_far, goal_type, goal_index


#########################################################
# make_door_dict
#########################################################
def make_door_dict(master_door_info, robot_loc, initial_loc):
    door_dict = {'all':np.array([]), 'left':np.array([]), 'right':np.array([]), 'left_sides':np.array([]), 'right_sides':np.array([]), 'all_sides':np.array([])}
    master_door_list_local, master_door_sides_local = master_door_info[:]
    if master_door_list_local.shape[0] > 1:
        path_theta = np.arctan2(robot_loc[0][1] - initial_loc[0][1], robot_loc[0][0] - initial_loc[0][0])
        door_thetas = np.arctan2(master_door_list_local[:,1] - initial_loc[0][1], master_door_list_local[:,0] - initial_loc[0][0])
        all_doors = master_door_list_local[np.logical_or(door_thetas > path_theta, door_thetas <= path_theta)]
        all_sides = master_door_sides_local[np.logical_or(door_thetas > path_theta, door_thetas <= path_theta)]
        left_doors = master_door_list_local[door_thetas > path_theta]
        left_sides = master_door_sides_local[door_thetas > path_theta]
        right_doors = master_door_list_local[door_thetas < path_theta]
        right_sides = master_door_sides_local[door_thetas < path_theta]
        left_doors_dist = np.sqrt((left_doors[:,0] - initial_loc[0][0])**2 + (left_doors[:,1] - initial_loc[0][1])**2)
        right_doors_dist = np.sqrt((right_doors[:,0] - initial_loc[0][0])**2 + (right_doors[:,1] - initial_loc[0][1])**2)
        all_doors_dist = np.sqrt((all_doors[:,0] - initial_loc[0][0])**2 + (all_doors[:,1] - initial_loc[0][1])**2)
        left_doors_idxs = np.argsort(left_doors_dist)
        right_doors_idxs = np.argsort(right_doors_dist)
        all_doors_idxs = np.argsort(all_doors_dist)
        door_dict['all'] = all_doors[all_doors_idxs]
        door_dict['all_sides'] = all_sides[all_doors_idxs]
        door_dict['left'] = left_doors[left_doors_idxs]
        door_dict['right'] = right_doors[right_doors_idxs]
        door_dict['left_sides'] = left_sides[left_doors_idxs]
        door_dict['right_sides'] = right_sides[right_doors_idxs]
    return door_dict    

#########################################################
# convert an (x, y) coordinate to the occupancy grid index
#########################################################
def convert_occupancy_grid_xy_coord_to_index(xy_coord, map_data):
    if xy_coord[1] >= map_data['height']:
        return None
    if xy_coord[0] >= map_data['width']:
        return None
    index = xy_coord[1] * map_data['width'] + xy_coord[0] # index = y * width + x
    return index

def convert_map_xy_to_occupancy_xy(map_xy, map_data):
    grid_x = int(round(((map_xy[0] - map_data['map_position_x']) / map_data['resolution'])))
    grid_y = int(round(((map_xy[1] - map_data['map_position_y']) / map_data['resolution'])))
    return [grid_x, grid_y]

######################################################################################
# compute whether point is near an obstacle
######################################################################################
def is_near_obstacle_or_unknown(loc, radius, map_data):
    loc = convert_map_xy_to_occupancy_xy(loc, map_data)       
    for x in range(loc[0]-radius, loc[0]+radius):
        for y in range(loc[1]-radius, loc[1]+radius):
            idx = convert_occupancy_grid_xy_coord_to_index([x,y], map_data)
            #print x,y, idx, map_data['size_of_map']
            if not idx == None and (map_data['occupancy_grid'][idx] > 60 or map_data['occupancy_grid'][idx] == -1):
                return True
    return False

#########################################################
# navigate_loop
#########################################################
def navigate_loop():
    global master_door_info, forward, door_info, initial_loc, robot_loc, run, image_global, doors_checked, frozen, num_forward_updates, forward_missing, state, map_data_global, internal_status, last_internal_status, door_label, door_number, initial_quaternion, door_direction, monitor_if_stuck, exhaustive, missing, state, command, robot_is_stuck, master_door_list, master_door_sides_list, all_doors, all_doors_sides    
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
    goal_found = False
    state = 'idle'
    prev_state = state
    ocr_im_count = 0
    while True:
        # service commands
        #print (internal_status, command)
        if command == 'stop' and not internal_status == 'standby':
            monitor_if_stuck = False
            movebase_stop()
            run = False
            frozen = True
            internal_status = 'standby'
        if command == 'start' and internal_status == 'standby':
            narration_pub.publish('NAVIGATE_DOOR')
            monitor_if_stuck = True
            master_door_list, master_door_sides_list, all_doors, all_doors_sides = np.array([]), np.array([]), [], []
            master_door_info = [master_door_list, master_door_sides_list]
            doors_checked = {'left':[], 'right':[], 'all':[]}
            door_info = [door_label, door_number, door_direction]
            num_forward_updates = 0
            forward_missing = 0
            print ("STARTING:", door_label, door_number, door_direction)
            initial_loc = get_robot_position()
            robot_loc = initial_loc
            exhaustive = False
            missing = False
            run = True
            frozen = False
            prev_goal = None
            state = 'idle'
            internal_status = 'working_on_it'
            robot_is_stuck = False

        # publish status
        if not internal_status == last_internal_status:
            log_message('internal_status: ' + internal_status)
            status.publish(internal_status)
            last_internal_status = internal_status
        
        # if internal status is anything besides 'working_on_it', sleep 
        if not internal_status == 'working_on_it':
            time.sleep(0.3)
            continue

        # Check if stuck
        if robot_is_stuck:
            log_message('robot is stuck!')
            movebase_stop()
            status.publish("failure")
            time.sleep(0.1)
            continue

        prev_state = state

        #grab robot position
        robot_loc = get_robot_position()
        
        #get information to determine next door to drive to
        door_label, door_number, door_direction = door_info

        if exhaustive:
            door_direction = 'all'
        #get travel distance
        travel_distance = np.sqrt((robot_loc[0][0] - initial_loc[0][0])**2 + (robot_loc[0][1] -initial_loc[0][1])**2)

        #if the robot has travelled more than 2.0 m, use its trajectory instead of angle for computing forward
        if travel_distance > 2.0:
            forward_angle = np.arctan2(robot_loc[0][1] - initial_loc[0][1], robot_loc[0][0] - initial_loc[0][0])
        else:
            _, _, forward_angle = tf.transformations.euler_from_quaternion(robot_loc[2])

        #compute forward direction
        this_forward = update_forward_goal(robot_loc, forward_angle, num_forward_updates)[0]

        #generate dictionary of door localizations
        door_dict = make_door_dict(master_door_info, robot_loc, initial_loc)

        #get doors for the current side of the hallway
        tmp_list = door_dict[door_direction+'_sides']

        #get list of all doors that have already been checked
        tmp_checked = np.array([x[0:2] for x in doors_checked[door_direction]])
        tmp_checked_readtags = [x[2] for x in doors_checked[door_direction] if x[2] is not None]


        #Determine state of operation

        if tmp_list.shape[0] > 0 and door_number < len(tmp_list) and this_forward is not None and not missed:

            checked_flag = True
            while checked_flag:
                if door_number < len(tmp_list):
                    goal_center, goal_close, goal_far, goal_type, goal_index = door_to_goal(door_number, door_dict, door_direction)
                else:
                    state = 'failure'
                    log_message("all doors have been checked, failure")
                    break
                
                if tmp_checked.shape[0] > 0:
                    distances = np.sqrt((tmp_checked[:,0] - goal_center[0])**2 + (tmp_checked[:,1] - goal_center[1])**2)
                    min_dist_idx = np.argmin(distances)
                    #if door has already been checked, just go to the next one
                    if distances[min_dist_idx] < 0.5 and doors_checked[door_direction][min_dist_idx][2][0] is not None:
                        log_message("door has already been checked")
                        checked_flag = True
                        door_number = door_number+1
                        door_info = [door_label, door_number, door_direction]
                    else:
                        checked_flag = False
                else:
                    checked_flag = False

            # compute distance to goal
            goal_dist = np.sqrt((robot_loc[0][0] - goal_center[0])**2 + (robot_loc[0][1] - goal_center[1])**2)
            
            #if the goal is close, just drive to it directly
            if goal_dist < 3.0:
                state = 'drive_door'
            else:
                state = 'drive_forward'

        elif door_dict[door_direction].shape[0] > 0 and door_number < len(tmp_list) and this_forward is not None:
            state = 'return_to_start'
                
        elif this_forward is not None:
            state = 'drive_forward'

        elif len(tmp_list) == tmp_checked.shape[0] and len(tmp_list) == len(tmp_checked_readtags) and this_forward is None and travel_distance > 1.0 and exhaustive:
            log_message("checked all doors and at the end of the hallway, failure")
            state = 'failure'
            internal_status = 'failure'

        elif travel_distance > 1.0 and this_forward is None:
            if not exhaustive:
                state = 'return_to_start'
            else:
                log_message("was in exhaustive search and got to the end of the hallway, failure")
                state = 'failure'
                internal_status = 'failure'

        else:
            state = 'forward_missing'

        # print out state if it changed
        if state != prev_state:
           log_message("Changing state to: "+state)
        
        #act upon determined state
        if state == 'failure':
            narration_pub.publish('Door was not found')
            log_message("door not found, failure, stopping")
            run = False
            internal_status = 'failure'
            continue
        elif state == 'return_to_start':
            narration_pub.publish('Reached end of the hallway, returning to start')
            log_message("got to the end of the hallway, going back")
            frozen = True
            return_theta = math.atan2(initial_loc[0][1] - robot_loc[0][1], initial_loc[0][0] - robot_loc[0][0])
            return_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, return_theta)
            movebase_client([robot_loc[0][0], robot_loc[0][1], return_quaternion], True)
            movebase_client([initial_loc[0][0], initial_loc[0][1], initial_loc[2]], True)
            door_info = [door_label, 0 , 'all']
            frozen = False
            exhaustive = True
            prev_goal = None
            num_forward_updates = 0
            missed = False

        elif state == 'forward_missing':
            forward_missing+=1
            if forward_missing > 200:
                log_message("forward missing for too long, failure")
                run = False
                internal_status = 'failure'
                continue

        elif state == 'drive_forward':
            forward_missing = 0
            forward_theta = math.atan2(this_forward[1] - robot_loc[0][1], this_forward[0] - robot_loc[0][0])
            forward_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, forward_theta)
            goal = [this_forward[0], this_forward[1], forward_quaternion]

            goal_dist = 0
            if prev_goal is not None:
                goal_dist = np.sqrt((robot_loc[0][0] - prev_goal[0])**2 + (robot_loc[0][1] - prev_goal[1])**2)

            if goal_dist < goal_thresh:
                log_message("driving to new forward goal")
                movebase_client(goal, False)
                num_forward_updates+=1
                prev_goal = goal

        elif state == 'drive_door':
            #log_message("goal is close, drive up")
            goal = goal_center
            goal_c = goal_close
            goal_f = goal_far
            text_locs = []
            this_map = map_data_global
            doortags = []
            if not is_near_obstacle_or_unknown([goal[0], goal[1]], 10, this_map):
                
                #print (goal)
                if goal_type is not None and goal_index is not None:
                    narration_pub.publish('Approaching door %d on the %s to read door tag'%(goal_index, goal_type))
                else:
                    narration_pub.publish('Approaching door to read door tag')
                log_message("Driving to door")
                # drive up to door
                frozen = True
                movebase_client(goal_c, True)
                image1, image2 = None, None
                # pan camera accordingly
                if goal_type == 'left':
                    set_pt(0.0, 30.0)
                    image1 = image_global
                    movebase_client(goal_f, True)
                    #set_pt(25.0, 30.0)
                    image2 = image_global
                else:
                    set_pt(180.0, 30.0)
                    image1 = image_global
                    movebase_client(goal_f, True)
                    #set_pt(155.0, 30.0)
                    image2 = image_global
                
                # read door
                comb_im = np.vstack((image1, image2))
                cv2.imwrite('%05d_ocr.png'%ocr_im_count, comb_im)
                ocr_im_count+=1
                text_locs = detect_text(comb_im)
                print ("all text that ocr found:")
                print (text_locs)
                set_pt(90.0, 0.0)
                frozen = False
                

                # set camera back to home
                
                doortags = get_doortag(text_locs)
                print ('text', doortags)

            #ascertain if goal is found and do some common sense reasoning
            goal_found, goal_num, goal_dir, value, parity, less_than, read_number, read_letter, total_tag = doortag_reasoning(door_info, doortags, goal_type)
            if total_tag!= '':
               narration_pub.publish('Found ' + total_tag)
            else:
               narration_pub.publish('No door tag read')
            print ('reasoning',goal_found, goal_dir, goal_num)

            if goal_found:
                narration_pub.publish('Door was found')
                internal_status = 'complete'
                run = False
                continue
            
            doors_checked[door_direction].append([goal[0], goal[1], [read_number, read_letter]])
            doors_checked['all'].append([goal[0], goal[1], [read_number, read_letter]])
            #if we haven't exceeded the number of checked doors for common sense, use the common sense predicted direction and number
            if len(doors_checked[door_direction]) < 2:
                #log_message("setting next door as goal")
                door_info = [door_label, goal_num, goal_dir]

            #if we have, check if the door was missed
            else:
                #log_message("check if door was missed")
                door_info = [door_label, door_number+1, goal_dir]
                if not exhaustive:
                    missed = check_missed(doors_checked, goal_dir, door_info[0])
                    if missed:
                        log_message("check if door was missed")

            prev_goal = None

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

######################################################################################
# update global image
######################################################################################
def update_global_image(data):
    global fresh_image, image_global
    fresh_image = True
    image_global = bridge.imgmsg_to_cv2(data, "bgr8")
    
######################################################################################
# update global map
######################################################################################
def update_map(mapdata):
    global map_data_global
    map_data = {}
    map_data['resolution']     = mapdata.info.resolution
    map_data['map_position_x'] = mapdata.info.origin.position.x
    map_data['map_position_y'] = mapdata.info.origin.position.y
    map_data['width']          = mapdata.info.width
    map_data['height']         = mapdata.info.height
    map_data['size_of_map']    = mapdata.info.width * mapdata.info.height
    map_data['occupancy_grid'] = mapdata.data # -1=unknown, 0=free_space, 100=obstacle
    map_data_global = map_data


#########################################################
# main callback
#########################################################
def callback(image, camera_info, velodyne):
    global all_doors, all_doors_sides, master_door_info, forward, robot_loc, image_global, frozen, map_data_global

    if not frozen:
        camera_model.fromCameraInfo(camera_info)
        assert isinstance(velodyne, PointCloud2)
        gen = pc2.read_points(velodyne)
        # collect the points
        points = []
        for p in gen:
            points.append(p)
        
        img = bridge.imgmsg_to_cv2(image, "bgr8")
        
        #image_global = img
        robot_loc = get_robot_position()
        door_detections, doors_boxes, robot_loc, scores, doors_median_list = get_doors(camera_model, img, points, robot_loc)
        for j in range(len(doors_boxes)):
            box, score = doors_boxes[j], scores[j]
            cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            cv2.putText(img, str(score), (int(box[2]), int(box[3])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        #cv2.imwrite('test.jpg', img)
        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        
        master_door_list = np.array([])
        master_door_sides_list = np.array([])
        master_door_list = np.array([])
        master_door_sides_list = np.array([])
        all_doors.extend(doors_median_list)
        all_doors_sides.extend(door_detections)

        if len(all_doors) > 1:
            #print (all_doors)
            door_locs_sides = np.array(all_doors_sides)
            door_locs1 = door_locs_sides[:,0,0:2]
            door_locs2 = door_locs_sides[:,1,0:2]
            door_locs = np.array(all_doors)
            door_clusters =  fclusterdata(door_locs, .5, criterion='distance') #input, cluster_thresh, criterion
            tmp_door_list = []
            tmp_door_sides_list = []
            for j in range(1, max(door_clusters)+1):
                if sum(door_clusters==j) > 2:
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
    
    #print (forward)
    
#########################################################
# service_request
#########################################################
def request(data):
    global operation, internal_status, door_label, door_number, door_direction, command

    # update command
    door_label     = str(data.door_tag)
    door_number    = data.door_index
    door_direction = data.door_dir
    command        = data.operation
    log_message("Got new command: " + command)

    # loop until the code has done the appropriate thing
    while True:
        if command == 'start' and not internal_status == 'standby':
            return doorTagResponse(True)
        if command == 'stop' and internal_status == 'standby':
            return doorTagResponse(True)
        time.sleep(0.1)


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
    separate_image_sub = rospy.Subscriber('/axis/image_raw_out', Image, update_global_image)
    image_pub = rospy.Publisher('/door_navigation/detections',   Image,  queue_size=10)
    cam_info_sub = message_filters.Subscriber('/axis/camera_info', CameraInfo)
    velodyne_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)
    map_sub   = rospy.Subscriber("/map", OccupancyGrid, update_map)
    axis_sub     = rospy.Subscriber('/axis/state', Axis, update_state)
    axis_pub     = rospy.Publisher('/axis/cmd', Axis, queue_size=1)
    narration_pub       = rospy.Publisher('/narration',     String, queue_size=1)
    s            = rospy.Service('/door_navigation/command', doorTag, request)
    status      = rospy.Publisher('/door_navigation/status', String, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, cam_info_sub, velodyne_sub], 10,.1)#, allow_headerless= True)
    ts.registerCallback(callback)

    # setup rviz marker publisher
    rviz_publisher = rviz_marker_publisher.setup()

    # wait one second, then set flag to start monitoring for robot being stuck
    rospy.Subscriber("/move_base/status", GoalStatusArray, move_base_callback, queue_size=1)
    rospy.Subscriber('/door_navigation/start_monitor_stuck', Bool, monitor_robot_stuck)
    monitor_stuck_pub = rospy.Publisher('/door_navigation/start_monitor_stuck',  Bool, queue_size=1)
    time.sleep(1)
    monitor_stuck_pub.publish(True)

    # wait for the publisher to settle, then go home
    time.sleep(1)
    go_home()


    navigate_loop()
    
    
    '''

    move_base_goal_status: 1
    Changing state to: drive_door
    Driving to door
    move_base_goal_status: 0
    move_base_goal_status: 1
    move_base_goal_status: 3
    Driving to door: done
    setting camera to desired angle
    setting camera to desired angle: done
    getting fresh image
    getting fresh image: done
    setting camera to desired angle
    setting camera to desired angle: done
    getting fresh image
    getting fresh image: done
    setting camera to desired angle
    setting camera to desired angle: done
    getting fresh image
    getting fresh image: done
    Reading text

    
    '''
    
    
    
