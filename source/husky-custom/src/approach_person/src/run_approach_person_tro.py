#!/usr/bin/env python

######################################################################################
# import libararies
######################################################################################
import os
import tf
import sys
import cv2
import math
import copy
import time
import rospy
import actionlib
import message_filters
import numpy as np
from scipy import stats
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
import approachable_person_detector
from image_geometry import PinholeCameraModel
from eyes.msg import PoseAndEyeData
from sensor_msgs.msg import Image, LaserScan, CameraInfo, PointCloud2
from approach_person.srv import *
import map_obj_into_3d_space
from my_axis_camera.msg import Axis
from sensor_msgs import point_cloud2
from std_msgs.msg import String, Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/robot-slang/husky-custom/src/rviz_marker_publisher/src/')
import rviz_marker_publisher

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
	global move_base_goal_status

	if len(data.status_list) == 0: 		# no goals
		move_base_goal_status = -2 		# indicates 'no goals'
	else:
		move_base_goal_status = data.status_list[-1].status
	#print 'move_base_goal_status', move_base_goal_status
 
 	'''
	if len(data.status_list) == 0: 		# no goals
		move_base_goal_status = -2 		# indicates 'no goals'
	elif len(data.status_list) > 1:		# 2+ goals
		move_base_goal_status = -1 		# indicates '2+ goals'
	else:
		for element in data.status_list:
			move_base_goal_status = element.status
	'''
	
######################################################################################
# process frame
######################################################################################
def process_look_forward(data):
	time0 = time.time()
	# get global variable + update frame index
	global frame_idx, tracking, last_person_detected_status

	# if we are not actively tracking people, save your processing power for other stuff
	if not tracking:
		return

	# update frame-related info
	frame_idx = frame_idx + 1

	# convert our image to cv2
	img = bridge.imgmsg_to_cv2(data.img, "bgr8")

	# detect people
	time1 = time.time()
	persons = approachable_person_detector.detect_people(img)
	time2 = time.time()

	# get people locations in map frame
	persons = get_people_locations_using_laserscan(data, persons, img) # takes ~0.005 seconds -> frame takes ~0.1 sec (10 fps)
	time3 = time.time()

	# determine if there is a person close enough
	person_detected = False
	for idx in persons.keys():
		if persons[idx]['closest_dist'] < args['person_flag_dist']:
			person_detected = True
			break
	
	# publish presence of person
	if not person_detected == last_person_detected_status: 
		last_person_detected_status = person_detected
		person_detected_pub.publish(person_detected)
		log_message('publishing person_detected: ' + str(person_detected))

	# track people
	approachable_person_detector.track_people(persons, frame_idx, data.velodyne_trans, data.header.stamp)
	time4 = time.time()
	#log_message('tracking people: ' + str(time4) + '.  message time stamp: ' + str(data.header.stamp))

	# get approachable_people
	approachable_person_detector.find_approachable_people(data.velodyne_trans)
	time5 = time.time()

	# draw results on frame
	annotated_img = approachable_person_detector.draw_tracks_on_frame(img, frame_idx)
	
	# publish frame
	frame_track_pub.publish(bridge.cv2_to_imgmsg(annotated_img, 'rgb8'))
	time6 = time.time()

	'''
	print 'frame processing time:         {0:0.2f}%, {1:0.5f}'.format((time6-time0)/(time6-time0), time6-time0)
	print '     detect_people:            {0:0.2f}%, {1:0.5f}'.format((time2-time1)/(time6-time0), time2-time1)
	print '     get_people_locations:     {0:0.2f}%, {1:0.5f}'.format((time3-time2)/(time6-time0), time3-time2)
	print '     track_people:             {0:0.2f}%, {1:0.5f}'.format((time4-time3)/(time6-time0), time4-time3)
	print '     find_approachable_people: {0:0.2f}%, {1:0.5f}'.format((time5-time4)/(time6-time0), time5-time4)
	print '     publish:                  {0:0.2f}%, {1:0.5f}'.format((time6-time5)/(time6-time0), time6-time5)
	'''

######################################################################################
# get the world location of this object using the current pose of the LIDAR + some trig
######################################################################################
def get_world_location_of_obj(trans, rot, dist_to_point, angle):
	# get the location of the point (in the baselink frame)
	x_rel = dist_to_point * math.cos(angle)
	y_rel = dist_to_point * math.sin(angle)

	# get the location of the point (in the map frame)
	combined_angle = angle+rot[2]
	x_dist = dist_to_point * math.cos(combined_angle)
	y_dist = dist_to_point * math.sin(combined_angle)

	# get the location of the point (in the map frame)
	x_prime = trans[0] + x_dist
	y_prime = trans[1] + y_dist
	'''
	print '\tangle:	        ', angle, '=', math.degrees(angle), 'degrees'
	print '\trot[2]:        ', rot[2], '=', math.degrees(rot[2]), 'degrees'
	print '\tcombined angle:', combined_angle, '=', math.degrees(combined_angle), 'degrees'
	print '\trobot_x: {0:0.3f}  robot_y: {1:0.3f}'.format(trans[0], trans[1])
	print '\tx_dist:  {0:0.3f}  y_dist:  {1:0.3f}'.format(x_dist, y_dist)
	print '\tx_prime: {0:0.3f}  y_prime: {1:0.3f}'.format(x_prime, y_prime)
	print '\tx_rel:   {0:0.3f}  y_rel:   {1:0.3f}'.format(x_rel, y_rel)
	'''
	return [[x_prime, y_prime], [x_rel, y_rel]]

######################################################################################
# get laserscan data for a certain angle range 
# the {min, max}_angles refer to the pan angle of the axis camera (and are given in 
# degrees).  The current HW configuration has the following mapping:
# 0 degrees = left.  90 degrees = straight forward.  180 degrees = right.
#
# The laserscan provides distances in meters for points around the circle.  The
# circle is currently constrained to [-2, 2], where the constraints are specified in
# radians.  The current HW configuration has the following mapping:
# pi/2 radians = left.  0 radians = straight forward.  -pi/2 radians = right.   
# The first item in laserscan.ranges corresponds to angle_min (i.e. the right-most point)
# It then moves counterclock-wise towards angle_max (i.e. the left-most point around the circle) 
######################################################################################
def get_laserscan_range(laserscan, min_angle, max_angle, trans, rot):
	# convert specified axis pan angles into radian limits:
	# 1) we need to swap the orientation from clockwise to counterclockwise (to follow the unit circle)
	# 2) need to account for the 90 degrees offset
	# 3) we need to convert into radians
	min_radian = math.radians(180.0-max_angle-90.0)
	max_radian = math.radians(180.0-min_angle-90.0)

	# debug
	#marker_id = 202020
	#this_marker  = {'id': marker_id, 'x': trans[0], 'y': trans[1], 'color': [0.0, 0.0, 1.0], 'scale': [0.4, 0.4, 0.1]}
	#point_marker = rviz_marker_publisher.create_cylinder_marker(this_marker)
	#rviz_marker_publisher.display_markers(rviz_publisher, [point_marker])

	# keep track of all locations within this range
	world_locs = []
	rel_locs   = []

	# loop over laserscan
	for i in range(0, len(laserscan.ranges)):
		angle = laserscan.angle_min + laserscan.angle_increment*i

		if angle > min_radian and angle < max_radian:
			# get world location of point
			dist_to_point = laserscan.ranges[i]
			if dist_to_point > args['pointcloud_max_dist']:
				continue
			world_loc, rel_loc = get_world_location_of_obj(trans, rot, dist_to_point, angle)
			world_locs.append(world_loc)
			rel_locs.append(rel_loc)

			# plot goal in rviz
			#marker_id += 1
			#this_marker  = {'id': marker_id, 'x': world_loc[0], 'y': world_loc[1], 'color': [1.0, 0.0, 0.0], 'scale': [0.2, 0.2, 0.1]}
			#point_marker = rviz_marker_publisher.create_cylinder_marker(this_marker)
			#rviz_marker_publisher.display_markers(rviz_publisher, [point_marker])

	return world_locs, rel_locs
	'''
	# compute projected points
	vel_points = np.array(rel_locs)

	# append column of zeros for lidar height at laserscan (not exactly the true height--just an estimate--but okay)
	zero_col = np.zeros((vel_points.shape[0], 1))
	vel_points = np.hstack((vel_points[:,0:2], zero_col))

	# append column of ones for transformation calculation (next)
	one_col = np.ones((vel_points.shape[0], 1))
	vel_points = np.hstack((vel_points[:,0:3], one_col))

	#trans = [-0.02532482, 0.28742033, -0.08615289]
	trans = [-0.03019076, 0.27373338, -0.05301323]
	#rot = [0.563540225425, -0.53427658701, 0.423247410108, 0.466725371859]
	rot = [0.544521193495, -0.512626565799, 0.452427950113, 0.485818509146]
	trans = tuple(trans) + ( 1,  )

	rotationMatrix = tf.transformations.quaternion_matrix( rot )
	# append translation to the last column of rotation matrix(4x4)
	rotationMatrix[ :, 3 ] = trans
	
	# transform 3D lidar points into 2D image frame 
	rotated_vel_points = rotationMatrix.dot(np.transpose(vel_points))
	uv_points = np.transpose(camera_model.P.dot(rotated_vel_points))
	uv_points = np.array(uv_points[:,0:2]/uv_points[:,2])

	# projected_points = these are the lidar points inside the object boxes
	projected_points = uv_points.astype(int)

	return world_locs, rel_locs, projected_points
	'''
	
######################################################################################
# get centroid point among locs using the lowest z-score
######################################################################################
def get_centroid_point(world_locs, rel_locs):
	# compute z-scores
	z = np.abs(stats.zscore(world_locs))

	# initialize filtered_locs with the point with the smallest z-score
	sum_z = [t[0] + t[1] for t in world_locs]
	lowest_z   = 100000
	lowest_idx = -1
	for idx in range(len(sum_z)):
		if sum_z[idx] < lowest_z:
			lowest_z   = sum_z[idx]
			lowest_idx = idx

	return world_locs[lowest_idx], rel_locs[lowest_idx]

######################################################################################
# check if a location is near other locations 
######################################################################################
def near_others(loc, other_locs, dist_thresh=0.2):
	for other_loc in other_locs:
		if dist(loc, other_loc) < dist_thresh:
			return True
	return False

######################################################################################
# filter laserscan data to not include outliers (e.g. one of the laserscan points 
# might go through a narrow doorway or window and be 3m away from all the other points)
######################################################################################
def filter_outliers(world_locs, rel_locs):
	# initialize filtered_locs with the point with the smallest z-score
	world_centroid_loc, rel_centroid_loc  = get_centroid_point(world_locs, rel_locs)
	filtered_world_locs = [world_centroid_loc]
	filtered_rel_locs   = [rel_centroid_loc]

	# add all points to filtered_world_locs that are near other points 
	while True:
		added = False
		del_idxs = []
		for idx in range(len(world_locs)):
			if near_others(world_locs[idx], filtered_world_locs):
				added = True
				filtered_world_locs.append(world_locs[idx])
				filtered_rel_locs.append(rel_locs[idx])
				del_idxs.append(idx)
		for idx in range(len(world_locs)-1,-1,-1):
			if idx in del_idxs:
				del world_locs[idx]
				del rel_locs[idx]
		if not added:
			break
	return filtered_world_locs, filtered_rel_locs

######################################################################################
# get people locations using laserscan
######################################################################################
def get_people_locations_using_laserscan(data, persons, cur_frame):
	# refer to global objects
   	global bridge, frame_idx
	
	# compute the pan angle of the left-most pixel
	pixel0_pan_angle = data.axis_pan - 65.6/2

	# compute the how many degrees (in the pan angle) each pixel in the image corresponds to.
	# there are 1920 pixels wide.  The camera FOV is 65.6 degrees.
	degrees_per_pixel = 65.6/1920.0

	# get the robot's position
	[trans, rot] = get_robot_position()

	# get tubes persons
	# convert persons list into dictionary
	# 	persons list structure: dictionary
	# 	persons[tube_id] = [x1 y1 x2 y2 score class_name]
	#	-->
	# 	all_persons[idx]['class_label']  = 'person'
	#	all_persons[idx]['temp_id']      = idx
	# 	all_persons[idx]['image_coords'] = [x1, y1, x2, y2] (of the latest frame)
	#   --> adds these fields -->
	#   all_persons[tube_id]['rel_2D_coords']         = [rel_x_dist_3d, rel_y_dist_3d] 
	#   all_persons[tube_id]['world_2D_coords']       = [world_x_dist_3d, world_y_dist_3d]
	#   all_persons[tube_id]['projected_points'][idx] = [x, y]
	#   all_persons[tube_id]['closest_dist']          = distance_in_meters
	all_persons = {}
	for idx in range(len(persons)):
		# get the bounding box
		x1, y1, x2, y2 = [int(i) for i in persons[idx][0:4]]

		# compute the pan angles of this object
		pan_angle_left  = pixel0_pan_angle + degrees_per_pixel * x1
		pan_angle_right = pixel0_pan_angle + degrees_per_pixel * x2
		
		# if the pan angles are too narrow, no lidar points will be detected.
		# if this is the case, add a small buffer.
		if pan_angle_right - pan_angle_left < 0.5:
			pan_angle_left  -= 0.25
			pan_angle_right += 0.25
		
		# get all the laserscan points of this object
		#world_locs, rel_locs = get_laserscan_range(data.laserscan, pan_angle_left, pan_angle_right, data.velodyne_trans, data.velodyne_rot)
		world_locs, rel_locs = get_laserscan_range(data.laserscan, pan_angle_left, pan_angle_right, trans, rot)
		#world_locs, rel_locs, projected_points = get_laserscan_range(data.laserscan, pan_angle_left, pan_angle_right, trans, rot)
		if world_locs == []:
			continue

		# filter out distant objects
		#filtered_world_locs, filtered_rel_locs = filter_outliers(world_locs, rel_locs)
		filtered_world_locs = world_locs
		filtered_rel_locs   = rel_locs
		#print 'filtered_rel_locs:', filtered_rel_locs
		if filtered_world_locs == []:
			continue
		
		# compute the closest point
		closest_idx  = None
		closest_dist = 10000.0
		for i in range(len(filtered_world_locs)):
			this_dist = dist(data.velodyne_trans, filtered_world_locs[i])
			if this_dist < closest_dist:
				closest_dist = this_dist
				closest_idx  = i
		
		# save data
		all_persons[idx] = {}
		all_persons[idx]['temp_id']               = idx
		all_persons[idx]['class_label']           = 'person'
		all_persons[idx]['image_coords']          = [x1, y1, x2, y2]
		all_persons[idx]['rel_2D_coords']         = filtered_rel_locs[closest_idx]
		all_persons[idx]['world_2D_coords']       = filtered_world_locs[closest_idx]
		all_persons[idx]['closest_dist']          = closest_dist
		#all_persons[idx]['projected_points']      = projected_points

	# annotate image
	annotated_img = approachable_person_detector.draw_objects_on_frame(cur_frame, all_persons, frame_idx)

	# publish frame with distances to each person
	frame_loc_pub.publish(bridge.cv2_to_imgmsg(annotated_img, 'rgb8'))

	# return all_persons
	return all_persons

######################################################################################
# keep track of when the goal is active
######################################################################################
def set_new_goal_active():
	# refer to global objects
	global new_goal_active
	new_goal_active = True

######################################################################################
# send 2D nav goal to robot
######################################################################################
def movebase_client(new_goal, person_location):
	# refer to global objects
	global new_goal_active
	

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

 	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, new_goal[2])
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]

	# plot goal in rviz
	this_marker = {'id': 6400, 'x': new_goal[0], 'y': new_goal[1], 'quaternion': quaternion, 'color': [0.0, 1.0, 1.0], 'scale': [1.0, 0.2, 0.2]}
	goal_marker = rviz_marker_publisher.create_arrow_marker(this_marker)
	this_marker = {'id': 6401, 'x': new_goal[0], 'y': new_goal[1], 'name': 'goal', 'color': [0.0, 1.0, 0.0], 'scale': 0.4}
	text_marker = rviz_marker_publisher.create_text_marker(this_marker)
	this_marker = {'id': 6402, 'x': person_location[0], 'y': person_location[1], 'color': [0.0, 0.0, 1.0], 'scale': [0.4, 0.4, 2.0]}
	person_marker = rviz_marker_publisher.create_cylinder_marker(this_marker)
	rviz_marker_publisher.display_markers(rviz_publisher, [goal_marker, text_marker, person_marker])

	# Sends the goal to the action server, wait for it to become active, then return control
	new_goal_active = False
	client.send_goal(goal, active_cb=set_new_goal_active)
	print 'sending new goal'
	while not new_goal_active:
		time.sleep(0.1)
	# wait for there to be one and only one goal before returning control
	# jared commented this out on 2020.07.01 during the debugging before the TRO dry runs/trials.
	# this while loop makes sense...but it takes a long time (>5 seconds sometimes)
	# that delay prevents ideal interaction with "approaching" people
	while move_base_goal_status in [-2, -1, 0]:
		rospy.sleep(0.1)
				
######################################################################################
# send cancelgoal to robot
######################################################################################
def movebase_stop():
	log_message('Stopping!')

	# Create an action client called "move_base" with action definition file "MoveBaseAction"
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.cancel_all_goals()
	
	#t1 = time.time()
	# wait for all goals to reach terminal state before returning control
	while move_base_goal_status in [-1, 0, 1, 6, 7, 9]:
		time.sleep(0.1)
	#t2 = time.time()
	#print 'cancel time:', t2-t1

	# delete goal markers
	delete_marker1 = rviz_marker_publisher.create_delete_marker(6400)
	delete_marker2 = rviz_marker_publisher.create_delete_marker(6401)
	delete_marker3 = rviz_marker_publisher.create_delete_marker(6402)
	rviz_marker_publisher.display_markers(rviz_publisher, [delete_marker1, delete_marker2, delete_marker3])

######################################################################################
# get robots position in the map frame
######################################################################################
def get_robot_position():
	# get robot's location
	now = rospy.Time.now()
	tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(1.0))
	(trans, quaternion) = tf_.lookupTransform( '/map', '/base_link', now )
	rot = tf.transformations.euler_from_quaternion(quaternion)
	return [trans, rot]

######################################################################################
# get goal using the person's location
######################################################################################
def get_goal(person_tube):
	# get robot's location
	[trans, rot] = get_robot_position()

	# get the location of the destination (in the map frame)
	x_dist  = person_tube['world_2D_coords'][-1][0] - trans[0]
	y_dist  = person_tube['world_2D_coords'][-1][1] - trans[1]
	h       = math.sqrt(x_dist**2 + y_dist**2)
	h_prime = h - args['stop_dist']
	theta_radians = math.atan2(y_dist, x_dist)
	theta_degrees = math.degrees(theta_radians)
	x_prime = trans[0] + h_prime * math.cos(theta_radians)
	y_prime = trans[1] + h_prime * math.sin(theta_radians)

	# get the desired end pose of the robot (in the map frame)
	# we get the relative angle change, add that to the current orientation angle,
	# and make sure it fits within {-pi, pi}
	x_dist  = person_tube['rel_2D_coords'][0]
	y_dist  = person_tube['rel_2D_coords'][1]
	theta_radians = math.atan2(y_dist, x_dist)
	start_pose_angle = rot[2]
	goal_pose_angle = start_pose_angle + theta_radians
	if goal_pose_angle > math.pi:
		goal_pose_angle = goal_pose_angle - (2.0 * math.pi)
	if goal_pose_angle < -math.pi:
		goal_pose_angle = goal_pose_angle + (2.0 * math.pi)

	# output positional information
	'''
	output_msg = "\trobot position:      ({0:0.2f}, {1:0.2f}).  pose: {2:0.2f}".format(trans[0], trans[1], rot[2])
	log_message(output_msg)
	output_msg = "\tperson position abs: ({0:0.2f}, {1:0.2f}).".format(person_tube['world_2D_coords'][-1][0], person_tube['world_2D_coords'][-1][1])
	log_message(output_msg)
	output_msg = "\tperson position rel: ({0:0.2f}, {1:0.2f}).".format(person_tube['rel_2D_coords'][0], person_tube['rel_2D_coords'][1])
	log_message(output_msg)
	output_msg = "\tgoal position:       ({0:0.2f}, {1:0.2f}).  pose: {2:0.2f}".format(x_prime, y_prime, goal_pose_angle)
	log_message(output_msg)
	'''

	'''
	opening_angle_radians = opening_angle * math.pi / 180
 	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, opening_angle_radians)
	'''
	goal = [x_prime, y_prime, goal_pose_angle]
	return goal

######################################################################################
# get distance between two points
######################################################################################
def dist(point1, point2):
	x_dist = point1[0] - point2[0]
	y_dist = point1[1] - point2[1]
 	return math.sqrt(x_dist**2 + y_dist**2)

######################################################################################
# get distance to location
######################################################################################
def get_robot_dist_to_location(location):
	# get robot's location
	[trans, rot] = get_robot_position()
	dist_to_location = dist(trans, location)
	return dist_to_location

######################################################################################
# approach person (in a closed loop fashion)
######################################################################################
def approach_person(person_id, type_person):
	# refer to global variables
	global args, command
	start_time = time.time()
	# some local variable
	goal = [1000000000, -1000000000]
	introduced_self = False
	engaged_human   = False
	person_tube     = None
	
	while True:
		# allow command to cancel loop
		if command == 'stop':
			log_message('\tcommanded to stop')
			return False

		# get person tube
		person_tube = approachable_person_detector.get_approachable_person(person_id)

		# if the person is still in view, and has moved far enough from orginal goal, issue new goal
		if not person_tube == None:
			new_goal = get_goal(person_tube)
			dist_between_goals = dist(goal, new_goal)
			if args['debug']:
				print "\tgoal:              ", goal
				print "\tnew_goal:          ", new_goal
				print "\tdist_between_goals:", dist_between_goals
			if dist_between_goals > args['new_goal_dist']:
				movebase_client(new_goal, person_tube['world_2D_coords'][-1])
				goal = new_goal

		# the robot has introduced itself.  now we may face a number of situations
		if introduced_self:
			# case 1: the person is directly in front of us...and the robot is now < stop_dist from the person.
			#		  this may occur if the person is walking up to us and stops before our goal.
			#		  (This prevents the robot from trying to make odd, fine-tuned adjustments to its position in the last few centimeters.) 
			if not person_tube == None and get_robot_dist_to_location(person_tube['world_2D_coords'][-1]) < args['stop_dist']:
				log_message('\twithin stop_dist')
				return True
			# case 2: the person is no longer visible (because our visibility up close is so poor) but the robot is still trying 
			#         to drive up to the person.  We continue in order to allow it to complete its navigation.
			#		  (This is because we want it to complete its driving route, then turn to face the person)
			elif move_base_goal_status in [0, 1, 2]:
				log_message('\tcompleting journey')
				time.sleep(0.1)
				continue
			# case 3: it has failed to drive up to the person
			elif move_base_goal_status in [4, 5, 6, 7, 8, 9]:
				log_message('\tfailed move_based status')
				return False
			# case 4: it has driven up to the person
			elif move_base_goal_status in [3]:
				log_message('\treached goal')
				return True

		# if we have not yet introduced ourselves, and our person is no longer approachable, return failure  
		if person_tube == None:
			log_message('\tperson no longer approachable')
			return False

		# if dealing with an approaching person, and they are still a ways off, wait until they are closer
		#if type_person == 'approaching' and not engaged_human and get_robot_dist_to_location(person_tube['world_2D_coords'][-1]) > args['engage_dist']:
		#	time.sleep(0.1)
		#	continue

		# if dealing with an approaching person, and they are close enough, then engage them
		if type_person == 'approaching' and not engaged_human and get_robot_dist_to_location(person_tube['world_2D_coords'][-1]) <= args['engage_dist']:
			narration_pub.publish('Engaging human for assistance.')
			log_message("\tengaging human for assistance")
			engaged_human = True
			mouth_pub.publish(args['engage_statement'])

		# when the robot is N feet away, engage the human in dialogue
		log_message("\trobot_dist_to_person: " + str(get_robot_dist_to_location(person_tube['world_2D_coords'][-1])))
		if not introduced_self and get_robot_dist_to_location(person_tube['world_2D_coords'][-1]) < args['introduce_dist']:
			narration_pub.publish('Introducing self to human.')
			log_message("\tintroducing self to human")
			introduced_self = True
			mouth_pub.publish(args['introduce_statement'])

		# let the robot drive for some amount of time before checking if the state has changed
		time.sleep(0.5)

		# if move_base has reached the goal, return success
		if move_base_goal_status == 3:
			log_message('\treached goal')
			return True

######################################################################################
# approach person loop
######################################################################################
def approach_person_loop():
	# refer to global variables
	global command, internal_status, tracking

	# some local variables
	last_internal_status       = ''
	last_app_person_in_view    = ''
	app_person_in_view         = None
	
	# loop forever
	while True:
		# service commands
		if command == 'stop' and not internal_status == 'standby':
			movebase_stop()
			internal_status = 'standby'
		if command == 'start' and internal_status == 'standby':
			narration_pub.publish('APPROACH PERSON')
			internal_status = 'working_on_it'
		
		# publish status
		if not internal_status == last_internal_status:
			log_message('internal_status: ' + internal_status)
			last_internal_status = internal_status
			status_publisher.publish(internal_status)
		
		# if we are suppose to be tracking people, determine whether there are any 
		if tracking:
			# print out what the status of our tubes are
			output_msg = approachable_person_detector.get_tubes_summary()
			if not output_msg == '':
				log_message(output_msg)
			
			# get the most approachable person
			person_id, type_person, person = approachable_person_detector.get_person_to_approach()
			if person_id >= 0:
				app_person_in_view = True
			else:
				app_person_in_view = False

			# publish finding
			if not app_person_in_view == last_app_person_in_view:
				last_app_person_in_view = app_person_in_view
				app_person_in_view_pub.publish(app_person_in_view)
				log_message('publishing app_person_in_view_pub: ' + str(app_person_in_view) + ' -> ' + str(person_id))

		# if internal status is anything besides 'working_on_it', sleep 
		if not internal_status == 'working_on_it':
			time.sleep(0.3)
			continue
	
		# if there is a person to approach, do so
		if tracking and person_id >= 0:
			log_message('starting to approach person')
			narration_pub.publish('Starting to approach person.')
			success = approach_person(person_id, type_person)

			# if you were able to approach them, publish success
			if success: 
				status_publisher.publish('complete')
				log_message('publishing approached_person: True')
				log_message( "successfully approached person.")
				narration_pub.publish("Successfully approached person.")
				internal_status = 'complete'
			# otherwise stop and see if there are other approachable people
			else: 
				movebase_stop()
				log_message("Person no longer approachable.  Checking if there are others.")
				narration_pub.publish('Person no longer approachable.  Checking if there are others.')
		# else there is not a person to approach, publish failure
		else:
			status_publisher.publish('failure')
			log_message('publishing approached_person: False')
			log_message('no people to approach!')
			narration_pub.publish('No people to approach.')
			time.sleep(0.1)

######################################################################################
# update the state of the camera
######################################################################################
def update_state(data):
	# refer to global objects
	global axis_state
	axis_state['pan']        = data.pan
	axis_state['tilt']       = data.tilt
	axis_state['zoom']       = data.zoom
	axis_state['brightness'] = data.brightness
	axis_state['focus']      = data.focus
	axis_state['iris']       = data.iris
	axis_state['autofocus']  = data.autofocus

######################################################################################
# get synchronized frame, laserscan, and robot pose (within 0.5 seconds of one another)
######################################################################################
def synchronized_frame_and_laserscan_and_robot_pose(image_sub, laserscan_sub):
	# refer to global objects
	global axis_state

	# if we are not looking forward, bail
	if not (axis_state['pan'] == 90.0 and axis_state['tilt'] == 0.0):
		return

	# compute where the robot was at when the image was taken
	# http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28Python%29
	now = rospy.Time.now()
	past = image_sub.header.stamp
	try:
		tf_.waitForTransformFull("/velodyne", now, "/map", past, "/map", rospy.Duration(0.5))
		(trans, quaternion) = tf_.lookupTransformFull("/map", now, "/velodyne", past, "/map")
		rot = tf.transformations.euler_from_quaternion(quaternion)
	except TransformException, e:
		log_message('Error getting transform:%s'%e)
		return

	# convert our image to cv2
	img = bridge.imgmsg_to_cv2(image_sub, "bgr8")

	# save off synchronized data
	sync_frame             = img
	sync_laserscan         = laserscan_sub
	sync_trans             = trans
	sync_rot               = rot
	fresh_synced_data_time = time.time()

	# setup PoseAndEyeData message
	msg                = PoseAndEyeData()
	msg.img            = bridge.cv2_to_imgmsg(sync_frame, 'rgb8')
	msg.laserscan      = sync_laserscan
	msg.velodyne_trans = sync_trans
	msg.velodyne_rot   = sync_rot
	msg.axis_pan       = 90.0
	msg.axis_tilt      = 0.0
	msg.header.stamp   = rospy.Time.now() 

	# process this message 
	process_look_forward(msg)

##############################################
# Update command (can be 'start', 'stop', 'start-tracking', 'stop-tracking')
# Status can be 'working_on_it', 'standby', 'complete', 'failure')
# if 'start', return True once the code has started working on it
# if 'stop', return True once the code has completely stopped 
##############################################
def command_callback(data):
	# refer to global objects
	global command, internal_status, tracking

	# update command
	command = data.command
	log_message("Got new command: " + command)

	# loop until the code has done the appropriate thing
	while True:
		if command == 'start':
			if not internal_status == 'standby':
				return commandNodeResponse(True)
		elif command == 'stop':
			if internal_status == 'standby':
				return commandNodeResponse(True)
		elif command == 'start-tracking':
			tracking = True
			return commandNodeResponse(True)
		elif command == 'stop-tracking':
			tracking = False
			# this may not be necessary.  It may be what causes thrashing between wander/approach person.
			# approachable_person detector automatically prunes the tubes based on time...so I don't think
			# we need an explicit reset.  Having it in there nukes the tubes data structure so that when control
			# is turned over to approachble person detector, there are no longer people to approach.
			#approachable_person_detector.reset()   
			return commandNodeResponse(True)
		else:
			log_message('RECEIVED INVALID COMMAND!')
			sys.exit()
			
		time.sleep(0.1)

######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	timestamp = datetime.now().strftime("%Y-%b-%d-%H-%M-%S")
	print timestamp,msg
	args['output_log'].write(timestamp + ' ' + msg + '\n')

######################################################################################
# when starting up...
######################################################################################
if __name__ == '__main__':
	#################################################################################################
	# global variables
	#################################################################################################
	axis_state            = {'pan': 90.0, 'tilt': 0.0}
	frame_idx             = 0
	camera_model          = PinholeCameraModel()
	move_base_goal_status = -2 # indicates 'no goals'
	command               = 'stop'
	internal_status       = 'standby'
	tracking              = False
	new_goal_active       = False
	last_person_detected_status = False

	# import launch file params 
	args = {}
	args['frame_width']           = rospy.get_param('approach_person/frame_width')
	args['frame_height']          = rospy.get_param('approach_person/frame_height')
	args['darknet_model_file']    = rospy.get_param('approach_person/darknet_model_file')
	args['darknet_weights_file']  = rospy.get_param('approach_person/darknet_weights_file')
	args['darknet_meta_file']	  = rospy.get_param('approach_person/darknet_meta_file')
	args['darknet_thresh']	      = rospy.get_param('approach_person/darknet_thresh')
	args['darknet_hier_thresh']   = rospy.get_param('approach_person/darknet_hier_thresh')
	args['darknet_nms']		      = rospy.get_param('approach_person/darknet_nms')
	args['input_width']		      = rospy.get_param('approach_person/input_width')
	args['input_height']		  = rospy.get_param('approach_person/input_height')
	args['gpu_id']		          = rospy.get_param('approach_person/gpu_id')
	args['pointcloud_max_dist']	  = rospy.get_param('approach_person/pointcloud_max_dist')
	args['pointcloud_min_dist']   = rospy.get_param('approach_person/pointcloud_min_dist')
	args['pointcloud_min_height'] = rospy.get_param('approach_person/pointcloud_min_height')
	args['pointcloud_max_height'] = rospy.get_param('approach_person/pointcloud_max_height')
	args['person_flag_dist']      = rospy.get_param('approach_person/person_flag_dist')
	args['same_object_thresh']    = rospy.get_param('approach_person/same_object_thresh')
	args['tracker_num_seconds']   = rospy.get_param('approach_person/tracker_num_seconds')
	args['forward_projection']    = rospy.get_param('approach_person/forward_projection')
	args['stationary_thresh']     = rospy.get_param('approach_person/stationary_thresh')
	args['move_towards_thresh']   = rospy.get_param('approach_person/move_towards_thresh')
	args['engage_dist']           = rospy.get_param('approach_person/engage_dist')
	args['introduce_dist']        = rospy.get_param('approach_person/introduce_dist')
	args['stop_dist']             = rospy.get_param('approach_person/stop_dist')
	args['engage_statement']      = rospy.get_param('approach_person/engage_statement')
	args['introduce_statement']   = rospy.get_param('approach_person/introduce_statement')
	args['new_goal_dist']         = rospy.get_param('approach_person/new_goal_dist')
	args['debug']                 = rospy.get_param('approach_person/debug')

	# open output log
	dateTimeObj        = datetime.now()
	output_filename    = dateTimeObj.strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
	home               = os.path.expanduser("~")
	log_path           = home + '/.ros/approach_person/log/'
	if not os.path.exists(log_path):
		os.makedirs(log_path)
	output_log         = open(log_path + output_filename, 'w')
	args['output_log'] = output_log
	log_message('Starting approach person')

	#################################################################################################
	# setup support stuff
	#################################################################################################
	approachable_person_detector.setup(args)

	# setup cv bridge
	bridge = CvBridge()

	# setup rviz marker publisher
	rviz_publisher = rviz_marker_publisher.setup()

	# get the transform
	tf_ = tf.TransformListener()

	#################################################################################################
	# intialize node and setup subscriptions/publications
	#################################################################################################
	try:
		# name our node
		rospy.init_node('approach_person')

		# stop any current goals in move_base
		movebase_stop()

		# setup publishers
		frame_track_pub        = rospy.Publisher('/approach_person/tracks',   Image,  queue_size=1)
		frame_loc_pub          = rospy.Publisher('/approach_person/location', Image,  queue_size=1)
		person_detected_pub    = rospy.Publisher("/approach_person/person_detected", Bool, queue_size=1)
		app_person_in_view_pub = rospy.Publisher('/approach_person/app_person_in_view', Bool, queue_size=1)
		approached_person_pub  = rospy.Publisher('/approach_person/approached_person', Bool, queue_size=1)
		status_publisher       = rospy.Publisher('/approach_person/status',   String, queue_size=1)
		mouth_pub              = rospy.Publisher('/mouth_and_ears/say', String, queue_size=1)
		narration_pub          = rospy.Publisher('/narration', String, queue_size=1)

		# setup subscriptions
		rospy.Subscriber('/axis/state', Axis, update_state)
		rospy.Subscriber("/move_base/status", GoalStatus, move_base_callback, queue_size=1)

		# setup synchronzied subscribers: http://docs.ros.org/kinetic/api/message_filters/html/python/
		image_sub     = message_filters.Subscriber('/axis/image_raw_out', Image)
		laserscan_sub = message_filters.Subscriber('/scan', LaserScan)
		ts = message_filters.ApproximateTimeSynchronizer([image_sub, laserscan_sub], 1, 0.5) # messages, queue_size, slop_in_sec
		ts.registerCallback(synchronized_frame_and_laserscan_and_robot_pose)

		# setup services
		rospy.Service('/approach_person/command', commandNode, command_callback)

		# loop forever
		approach_person_loop()
	except rospy.ROSInterruptException: 
		pass

