#!/usr/bin/env python

import rospy
from dialog.srv import *
from gesture_detect.srv import *
from door_detection.srv import *
from ocr.srv import *
from door_navigation.srv import *
from intersection_mapper.srv import singleRoute
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from intersection_mapper.msg import Routes
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import re
import tf
import math
import time
import pyttsx3
door_label = None
debug = False
run = False
door_number = 0
door_direction = None
forward = []
forward_latched = False
map_data = {}
map_data_latched = False
speech_engine = None
tf_ = tf.TransformListener()
sys.path.append('/home/tilyevsk/robot-slang/husky-custom/src/rviz_marker_publisher/src/')
import rviz_marker_publisher

#########################################################
# publish doors in rvi
#########################################################
def publish_door_lines(map_locations, names):
    # keep track of rviz markers
    my_markers = []
    # delete existing markers
    marker_id = 1000
        #loop over door points
    for i,point in enumerate(map_locations):
                #print point
        marker = {'id': marker_id, 'points':point, 'scale': .2, 'color': [0.0, 0.0, 1.0]}
        line_marker = rviz_marker_publisher.create_line_strip_marker(marker)    
        my_markers.append(line_marker)
                
        marker_id += 1
        #name = '({0:.2f}'.format(point[1]) + ', {0:.2f})'.format(point[2])

        text_marker = rviz_marker_publisher.create_text_marker(marker)
        my_markers.append(text_marker)
        marker_id += 1

    # display all the sphere markers
    rviz_marker_publisher.display_markers(marker_pub, my_markers)
######################################################################################
# get robots position in the map frame
######################################################################################
def get_robot_position():
    # get robot's location
    now = rospy.Time.now()
    tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(0.1))
    (trans, quaternion) = tf_.lookupTransform( '/map', '/base_link', now )
    rot = tf.transformations.euler_from_quaternion(quaternion)
    return [trans, rot, quaternion]
    
######################################################################################
# send 2D nav goal to robot
######################################################################################
def interpret_map(data):
    # extract some map data
    global map_data, map_data_latched
    if not map_data_latched:
        map_data = {}
        map_data['resolution']       = data.info.resolution
        map_data['map_position_x'] = data.info.origin.position.x
        map_data['map_position_y'] = data.info.origin.position.y
        map_data['width']           = data.info.width
        map_data['height']           = data.info.height
        map_data['size_of_map']       = data.info.width * data.info.height
        map_data['occupancy_grid'] = data.data
                
        map_data['map']   = np.zeros((map_data['height'], map_data['width']), dtype=int)
        # populate map_ with occupancy_grid but categorize cells to one of three options:
        # -1  = unknown
        # 0   = free space
        # 100 = obstacle 
        for i in range(map_data['size_of_map']):
            y = i / map_data['width']
            x = i % map_data['width']
            # if the cell is unknown, keep it as -1 (unknown)
            if map_data['occupancy_grid'][i] == -1:
                map_data['map'][y][x] = -1
            # if the cell has an uncertain score, set it to -1 (unknown)
            elif map_data['occupancy_grid'][i] > 40 and map_data['occupancy_grid'][i] < 60:
                map_data['map'][y][x] = -1
            # if the cell is likely to be an obstacle, set it to 100 (obstacle)
            elif map_data['occupancy_grid'][i] >= 60:
                map_data['map'][y][x] = 100
            # else set them to free space
            else:
                map_data['map'][y][x] = 0
        #map_data['map'] = map_        

def check_goal(point, map_data):
    #compute location in occupancy grid
    grid_x = int(((point[0] - map_data['map_position_x']) / map_data['resolution']))
    grid_y = int(((point[1] - map_data['map_position_y']) / map_data['resolution']))
    #check if no obstacles too close
    for i in range(grid_x-5,grid_x+6):
        for j in range(grid_y-5,grid_y+6):
            if i in range(map_data['width']) and j in range(map_data['height']):
                cell = map_data['map'][j][i]
                if cell == 100:
                    return False
    return True
def movebase_client(new_goal,close, door):#, person_location):
    #print "Issuing new goal to x:", new_goal[0], "   y:", new_goal[1], "  angle:", new_goal[2]
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    if door:
        if close:
            goal.target_pose.pose.position.x = new_goal.x_close
            goal.target_pose.pose.position.y = new_goal.y_close
        else:
            goal.target_pose.pose.position.x = new_goal.x_far
            goal.target_pose.pose.position.y = new_goal.y_far

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = new_goal.z_ori
        goal.target_pose.pose.orientation.w = new_goal.w_ori
    else:
        goal.target_pose.pose.position.x = new_goal[0]
        goal.target_pose.pose.position.y = new_goal[1]
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = new_goal[2][2]
        goal.target_pose.pose.orientation.w = new_goal[2][3]
    #print (goal)
    #if not debug:
    # Sends the goal to the action server, then return control
    #print ('sending goal')
    client.send_goal(goal)
    # plot goal in rviz
    #this_marker = {'id': 6400, 'x': goal.target_pose.pose.position.x, 'y': goal.target_pose.pose.position.y, 'quaternion': [0.0,0.0,goal.target_pose.pose.orientation.z ,goal.target_pose.pose.orientation.w], 'color': [0.0, 1.0, 1.0], 'scale': [1.0, 0.2, 0.2]}
    #goal_marker = rviz_marker_publisher.create_arrow_marker(this_marker)
    #this_marker = {'id': 6401, 'x': new_goal[0], 'y': new_goal[1], 'name': 'goal', 'color': [0.0, 1.0, 0.0], 'scale': 0.4}
    #text_marker = rviz_marker_publisher.create_text_marker(this_marker)
    #rviz_marker_publisher.display_markers(marker_pub, [goal_marker, text_marker])

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()

######################################################################################
# send cancelgoal to robot
######################################################################################
def movebase_stop():
    print "Stopping!"
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.cancel_all_goals()
    # delete goal markers
    delete_marker1 = rviz_marker_publisher.create_delete_marker(6400)
    delete_marker2 = rviz_marker_publisher.create_delete_marker(6401)
    rviz_marker_publisher.display_markers(marker_pub, [delete_marker1, delete_marker2])

def door_tag(text):
    for item in text:
        if item[-1] != '\n' and (len(item) == 2 or len(item) == 3 or len(item) == 4):# and item[0] in ['1', '2', '3']:
            #print('door tag ' + item + ' recorded')
            return [item] 
    return []
def get_possible_door_tags(text): 
    all_tags = []
    for item in text:
        # find all instances of 2-3 digits followed by 0-1 alphabetical characters 
        possible_tags = re.findall(r"[0-9]{2,3}[a-zA-Z]{0,1}", item)
        all_tags.extend(possible_tags)
    return all_tags

def forward_callback(data):
    global forward, forward_latched, initial_quaternion, recenter, run
    if not forward_latched and run:
        #data_items = [data.x_world[0], data.y_world[0], data.x_rel[0], data.y_rel[0], data.direction[0], data.intersection]
        #x_world = data.x_rel[0]/4
        #y_world = data.y_rel[0]
        if len(data.x_rel) == 0:
            forward = []
        else:
            [trans, rot, quaternion] = get_robot_position()
            x_dist = data.x_rel[0]
            y_dist = data.y_rel[0]
            theta = math.atan2(y_dist, x_dist)
            quaternion = tf.transformations.quaternion_from_euler(0.0,0.0, theta)
            forward = [math.cos(theta) + trans[0], math.sin(theta) + trans[1], initial_quaternion]
            recenter = [x_dist*6/8 + trans[0], y_dist + trans[1], initial_quaternion]

######################################################################################
# get goal using the possible route location
######################################################################################
def get_goal(route):
    # get robot's location
    [trans, rot, quaternion] = get_robot_position()

    x_dist = route['rel_coords'][0]
    y_dist = route['rel_coords'][1]
    theta = math.atan2(y_dist, x_dist)
    quaternion = tf.transformations.quaternion_from_euler(0.0,0.0, theta)
    goal = [math.cos(theta) + trans[0], math.sin(theta) + trans[1], quaternion]
    return goal
def retrieve_forward_goal():
    # incrementally get a wider and wider forward window
    for forward_range in [[345, 15], [330, 30], [315, 45], [300, 60], [285, 75]]: 
        try:
            single_route_srv = rospy.ServiceProxy('/intersection_mapper/single_direction', singleRoute)
            data = single_route_srv(forward_range[0], forward_range[1])
        except rospy.ServiceException, e:
            print "\t\tService call failed: %s"%e    
            return [-1, None]

        # if data found, break
        if len(data.x_world) > 0:
            break
        
    # if no data, return failure
    if len(data.x_world) == 0:
        return []

    forward = {}
    forward['world_coords'] = [data.x_world[-1], data.y_world[-1]]
    forward['rel_coords']   = [data.x_rel[-1],   data.y_rel[-1]]
    goal = get_goal(forward)
     #quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, goal[2])
    #forward['goal'] = goal
    #forward['quaternion'] = quaternion 
    print goal
    return goal
def door_nav():
    global door_label, door_number, door_direction, forward, forward_latched, map_data, map_data_latched, direction, recenter, run
    text = []
    doors_checked = []
    doors_checked_coords = []
    door_tag_label = []
    while door_label not in text:
        if not run:
            time.sleep(1)
            continue
        status.publish('working on it')
        rospy.wait_for_service('door_service')
        door_detection = rospy.ServiceProxy('door_service', doorCoords)
        door_request = door_detection(door_direction, door_number)
        print (door_number, door_request.x_close, door_request.y_close)
        [cur_pose, _,_] = get_robot_position()
        dist_from_cur_pos = math.sqrt((cur_pose[0] - door_request.x_close)**2 + (cur_pose[1] - door_request.y_close)**2)
        #print ('distance from closest door: %f'%(dist_from_cur_pos))
        #if door_request.valid: 
         #   publish_door_lines([[[door_request.door_x_close, door_request.door_y_close], [door_request.door_x_far, door_request.door_y_far]]])
        if door_request.valid == False or dist_from_cur_pos > args['door_dist']:
            
            time.sleep(2)
            #do something else if door request is invalid like drive straight or something, subscribe to intersection mapper
            forward_latched = True
            forward_copy = retrieve_forward_goal() #instead of forward
            forward_latched = False
            print forward_copy
            if len(forward_copy) == 0:
                status.publish('failure')
                print ("Failure - cannot move forward.")
                nar_pub.publish("Failure - cannot move forward.")
                run = False
                continue
            if door_request.valid == False: 
                print ('No doors available.  Driving straight.')
                nar_pub.publish('No doors available.  Driving straight.')
            else:
                print ('Too far from door: {0:0.2f}m.  Driving straight.'.format(dist_from_cur_pos))
                nar_pub.publish('Too far from door: {0:0.2f}m.  Driving straight.'.format(dist_from_cur_pos))
                
                #forward_copy[2][2] = door_request.z_ori
                #forward_copy[2][3] = door_request.w_ori
            
            #print (forward)
            door_stop = door_detection('stop_detect', door_number)
            move_request = movebase_client(forward_copy, True, False)
            door_start = door_detection('resume_detect', door_number)
            time.sleep(2)
        else:
            if door_direction == 'unknown':
			            
                print ('Near door: {0:0.2f}m. Driving up to it.'.format(dist_from_cur_pos))
                nar_pub.publish('Near door: {0:0.2f}m. Driving up to it.'.format(dist_from_cur_pos))
            else:
                print ('Driving up to next door on the ' + door_direction + '.')
                nar_pub.publish('Driving up to next door on the ' + door_direction + '.')
            #call move base client on door close point
            map_data_latched = True
            #check that point is unobstructed
            if check_goal([door_request.x_close, door_request.y_close],map_data):
                door_stop = door_detection('stop_detect', door_number)
                move_request = movebase_client(door_request, True, True)
                door_start = door_detection('resume_detect', door_number)
                rospy.wait_for_service('text_service')
                text_detection= rospy.ServiceProxy('text_service', textDetect)
                text_resp = text_detection(door_request.direction)
                text = text_resp.text
                door_tag_label = get_possible_door_tags(text)
                #doors_checked.extend(door_tag_label)
            #print door_tag_label
            if door_label not in text and door_tag_label == []:
                #repeat move request on door far point
                #check that point is unobstructed
                if door_tag_label == []:
                    print ('No door tag detected.  Trying other side of door.')
                    nar_pub.publish('No door tag detected.  Trying other side of door.')
                if check_goal([door_request.x_far, door_request.y_far],map_data):
                    door_stop = door_detection('stop_detect', door_number)
                    move_request = movebase_client(door_request, False, True)
                    door_start = door_detection('resume_detect', door_number)
                    rospy.wait_for_service('text_service')
                    text_detection= rospy.ServiceProxy('text_service', textDetect)
                    text_resp = text_detection(door_request.direction)
                    text = text_resp.text
                    door_tag_label = get_possible_door_tags(text)
                    #doors_checked.extend(door_tag_label)
            map_data_latched = False
            #print door_tag_label
                
            doors_checked_coords.append([[door_request.door_x_close, door_request.door_y_close], [door_request.door_x_far, door_request.door_y_far]])
            
            print ("Potential Door Tags:", door_tag_label)
                
            if door_tag_label != []:
                if not door_tag_label[0].isdigit():
                    print ("Door tag label has alphabetical characters in it.  Cannot parse out room number.")
                    nar_pub.publish("Door tag label has alphabetical characters in it.  Cannot parse out room number.")
                else:
                    print ('Room number ' + door_tag_label[0] + ' found.')
                    print ('publishing this fact')
                    nar_pub.publish('Room number ' + str(door_tag_label[0]) + ' found.')
                    doors_checked.append(str(door_tag_label[0]))
                    if len(door_tag_label)>2: 
                        door_tag_num = int(door_tag_label[0][0:3])
                        door_label_int = int(door_label[0:3])
                    else:
                        door_tag_num = int(door_tag_label[0])
                        door_label_int = int(door_label)
                    if (door_tag_num % 2) != (door_label_int % 2):
                        door_direction = 'left' if door_request.direction == 'right' else 'right'
                        print ("I'm on the wrong ("+door_request.direction+") side of the hallway.  Will now check doors on the " + door_direction + " side.")
                        nar_pub.publish("I'm on the wrong ("+door_request.direction+") side of the hallway.  Will now check doors on the " + door_direction + " side.")
                    else:
                        door_direction = door_request.direction
                        door_label_diff = abs(int(door_label[0:3]) - door_tag_num)
                        if door_label_diff ==0:
                            door_number+=1
                        else: 
                            door_number+= door_label_diff/2
                        print ('Incrementing door number, it is now: %d.'%(door_number))
                        nar_pub.publish('Now looking for door number %d on '%(door_number+1)+ door_direction + " side.")
            else:
                print('No door tag read, looking for next door.')
                nar_pub.publish('No door tag read, looking for next door.')
                #door_direction = door_request.direction
                door_number+=1
                doors_checked.append('unk')
            publish_door_lines(doors_checked_coords, doors_checked)
            #print gesture_request, door_request, text
        if door_label in text:
            print "Room " + door_label + " found."
            nar_pub.publish("Room " + door_label + " found.")
            status.publish('complete')
        print ("I visited %d doors:"%(door_number+1), doors_checked)
        
        #print (doors_checked)
    
def request(data):
    global door_label, door_number, initial_quaternion, door_direction, run
    if data.operation == 'start':
       
        door_label = str(data.door_tag)
        door_number = data.door_index
        door_direction = data.door_dir
        nar_pub.publish('NAVIGATE DOOR')
        print ("STARTING:", door_label, door_number, door_direction)
        [_, _, initial_quaternion] = get_robot_position()
        rospy.wait_for_service('door_service')
        door_detection = rospy.ServiceProxy('door_service', doorCoords)
        door_request = door_detection('detect',0)
        run = True
        return doorTagResponse(True)
    if data.operation == 'stop':
        movebase_stop()
        run = False
        rospy.wait_for_service('door_service')
        door_detection = rospy.ServiceProxy('door_service', doorCoords)
        door_request = door_detection('stop_detect',0)
        return doorTagResponse(False)
    
if __name__== "__main__":
    #global door_label, door_number, initial_quaternion, direction 
    args = {}
    args['door_dist'] = rospy.get_param('door_navigation/door_dist')
    rospy.init_node('door_navigation')
    forward_sub = rospy.Subscriber('intersection_mapper/forward', Routes, queue_size=1, callback = forward_callback)
    map_sub     = rospy.Subscriber('/map', OccupancyGrid, queue_size=1, callback = interpret_map)
    s           = rospy.Service('/door_navigation/command', doorTag, request)
    nar_pub     = rospy.Publisher('/narration', String, queue_size=1)
    status      = rospy.Publisher('/door_navigation/status', String, queue_size=1)
    marker_pub  = rospy.Publisher('door_markers', MarkerArray, queue_size=1)
    door_nav()
    rospy.spin()
    
