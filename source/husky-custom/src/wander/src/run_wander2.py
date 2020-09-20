#!/usr/bin/env python

#########################################################
# import libraries
#########################################################
import os
import tf
import sys
import math
import json
import time
import rospy
import random
import actionlib
from datetime import datetime
from wander.srv import *
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from intersection_mapper.msg import Routes2
from intersection_mapper.srv import singleRoute, recoveryRoutes
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
	global move_base_goal_status, last_move_base_goal_status

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

	#if not move_base_goal_status == last_move_base_goal_status:
	#	log_message('move_base_goal_status: ' + str(move_base_goal_status))
	#	last_move_base_goal_status = move_base_goal_status
			
######################################################################################
# send cancelgoal to robot
######################################################################################
def movebase_stop():
	log_message('Stopping!')
	
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
def movebase_client(new_goal, quaternion):
	# refer to global objects
	global new_goal_active

	# Create an action client called "move_base" with action definition file "MoveBaseAction"
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

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
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]

	# plot goal in rviz
	this_marker = {'id': 6400, 'x': new_goal[0], 'y': new_goal[1], 'quaternion': quaternion, 'color': [0.0, 1.0, 1.0], 'scale': [1.0, 0.2, 0.2]}
	goal_marker = rviz_marker_publisher.create_arrow_marker(this_marker)
	this_marker = {'id': 6401, 'x': new_goal[0], 'y': new_goal[1], 'name': 'goal', 'color': [0.0, 1.0, 0.0], 'scale': 0.4}
	text_marker = rviz_marker_publisher.create_text_marker(this_marker)
	rviz_marker_publisher.display_markers(rviz_publisher, [goal_marker, text_marker])

	# Sends the goal to the action server, waits for it to become active, then return control
	new_goal_active = False
	client.send_goal(goal, active_cb=set_new_goal_active)
	while not new_goal_active:
		time.sleep(0.1)
	# wait for there to be one and only one goal before returning control
	while move_base_goal_status in [-2, -1, 0]:
		time.sleep(0.1)

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

def set_forward_goal(cur_goal, new_goal, quaternion, num_fwd_updates):
	# if you are getting near the goal, and another goal is further up ahead, update it to be further ahead 
	if not new_goal == None and (cur_goal == None or (get_robot_dist_to_location(cur_goal) < args['goal_update_dist'] and dist(cur_goal, new_goal) > 1.0)):
		movebase_client(new_goal, quaternion)
		return [new_goal, num_fwd_updates+1]
	else:
		return [cur_goal, num_fwd_updates]

def update_forward_goal(cur_goal, forward_angle, num_fwd_updates):
	# get robot position
	[trans, rot] = get_robot_position()
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
		cur_goal, num_fwd_updates = set_forward_goal(cur_goal, furthest_goal, furthest_quaternion, num_fwd_updates)
	# update our forward goal as normal
	else:
		new_goal, new_quaternion = get_forward_goal(trans, forward_angle, num_fwd_updates)
		cur_goal, num_fwd_updates = set_forward_goal(cur_goal, new_goal, new_quaternion, num_fwd_updates)

	# return our current goal and how many times we have updated
	return cur_goal, num_fwd_updates

######################################################################################
# get a rotate goal in the map frame
# -90 = turn right by 90 degrees
#  90 = turn left by 90 degrees 
######################################################################################
def get_rotate_goal(degrees):
	# get robot's location
	[trans, rot] = get_robot_position()
	
	# get desired angle of rotation
	robot_angle_degrees = math.degrees(rot[2])
	theta_radians = math.radians(robot_angle_degrees + degrees)

	# return goal	
	goal = [trans[0], trans[1], theta_radians]
	return goal
	
#########################################################
# execute rotate 
#########################################################
def rotate(turn_direction):
	log_message('issuing rotate command!')
	# determine the amount of turning to make
	if   turn_direction == 'forward-left':  goal = get_rotate_goal(45)
	elif turn_direction == 'left':          goal = get_rotate_goal(90)
	elif turn_direction == 'back-left':     goal = get_rotate_goal(135)
	elif turn_direction == 'turn-around':   goal = get_rotate_goal(180)
	elif turn_direction == 'back':          goal = get_rotate_goal(180)
	elif turn_direction == 'back-right':    goal = get_rotate_goal(225)
	elif turn_direction == 'right':         goal = get_rotate_goal(270)
	elif turn_direction == 'forward-right': goal = get_rotate_goal(315)
	else:                                   goal = get_rotate_goal(turn_direction)
	# get the quaternion and issue the command!
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, goal[2])
	movebase_client(goal, quaternion)

#########################################################
# if wander gets stuck, we have it perform a 360, hoping this creates a clear path
# we do this by using cmd_vel to spin around for N seconds.
#########################################################
def rotate_recovery():
	# refer to global variables
	global command, fresh_routes
	
	log_message('Spinning in a 360.')

	start_time = time.time()
	rotation_vel = 0.3
	rotation_time_in_sec = ((2 * math.pi) / rotation_vel) - 1.0 # -1.0 based on empirical observation to get it to rotate closer to 360 degrees.
	
	# setup twist message
	vel_msg = Twist()
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = rotation_vel

	while True:
		# bail if told to stop
		if command == 'stop':
			break 
		elapsed_time = time.time() - start_time
		if elapsed_time > rotation_time_in_sec:
			break
		cmd_vel_pub.publish(vel_msg)
		time.sleep(0.1)

	# wait until we get a new set of routes available to the robot
	fresh_routes = False
	while not fresh_routes:
		time.sleep(0.1)

#########################################################
# get random direction to drive (forward, left, right) 
#########################################################
def get_random_direction():
	global routes
	for path_label in routes['paths'].keys():
		path_active, direction, x, y = routes['paths'][path_label] 
		if path_active and direction in ['forward', 'left', 'right']:
			return [path_label, direction]
	# fail case in the event that we don't have a forward/left/right
	for path_label in routes['paths'].keys():
		path_active, direction, x, y = routes['paths'][path_label]
		if direction == 'back':
			return [path_label, direction]
	# ultimate failure case (we should never get here)
	return [None, None]

#########################################################
# get_direction_to_go
#########################################################
def get_direction_to_go():
	global int_paths_taken, routes

	# int_paths_taken[intersection_id][path_label] = time_last_visited 
	now = time.time()

	# if we have been to this intersection before, get the oldest path visited
	if routes['intersection_id'] in int_paths_taken:
		# Add any new path_labels 
		for path_label in routes['paths'].keys():
			if not path_label in int_paths_taken[routes['intersection_id']].keys():
				int_paths_taken[routes['intersection_id']][path_label] = None
			# save off the entry path (as being slightly in the past)
			if path_label == routes['entry_path_label']:
				int_paths_taken[routes['intersection_id']][path_label] = now - 1

		# initialize the oldest path
		oldest_path_label = None
		oldest_path_dir   = None
		oldest_path_time  = None
		
		# loop over the possible paths
		for path_label in int_paths_taken[routes['intersection_id']].keys():
			# skip over any paths that no longer exist or are inactive
			if not path_label in routes['paths'].keys():
				continue
			path_active, direction, x, y = routes['paths'][path_label]
			if not path_active:
				continue

			# if we've never visited this path and it is active, take it!
			time_last_visited = int_paths_taken[routes['intersection_id']][path_label]
			if time_last_visited == None:
				int_paths_taken[routes['intersection_id']][path_label] = now
				return direction
			# else keep track of the oldest path
			else:
				time_elapsed = abs(now - time_last_visited)
				if oldest_path_time == None or time_elapsed > oldest_path_time:
					oldest_path_time  = time_elapsed
					oldest_path_dir   = direction
					oldest_path_label = path_label

		# update and return the oldest path
		int_paths_taken[routes['intersection_id']][oldest_path_label] = now
		return oldest_path_dir

	# this is a new intersection, get a random direction
	else:
		exit_path_label, new_direction = get_random_direction()
		# fall back case in event of failure
		if exit_path_label == None:
			return 'back'
		
		# save off this int_paths_taken
		int_paths_taken[routes['intersection_id']] = {}
		for path_label in routes['paths'].keys():
			path_active, direction, x, y = routes['paths'][path_label]
			# save off the entry path (as being slightly in the past)
			if path_label == routes['entry_path_label']:
				int_paths_taken[routes['intersection_id']][path_label] = now - 10
			# save off our new direction path (as happening "now")
			elif path_label == exit_path_label:
				int_paths_taken[routes['intersection_id']][path_label] = now
			# save off the path_label as not having been visited (None)
			else:
				int_paths_taken[routes['intersection_id']][path_label] = None

		# return the direction we are taking
		return new_direction

#########################################################
# get directions available in intersection that are not backwards
#########################################################
def get_intersection_dirs():
	global routes
	intersection_directions = []
	for path_label in routes['paths'].keys():
		path_active, direction, x, y = routes['paths'][path_label]
		if path_active and direction in ['forward', 'left', 'right']:
			intersection_directions.append(direction)
	return intersection_directions

#########################################################
# get possible directions (that are greater than 2 m away)
#########################################################
def get_possible_directions():
	global routes
	possible_directions = []
	for dir_name in routes['dirs'].keys():
		for path_len in routes['dirs'][dir_name].keys():
			if float(path_len) > 2.0:
				if dir_name in possible_directions:
					break
				possible_directions.append(dir_name)
	return possible_directions

#########################################################
# get_recovery_direction
#########################################################
def get_recovery_direction():
	try:
		recovery_routes_srv = rospy.ServiceProxy('/intersection_mapper/get_recovery_angle', recoveryRoutes)
		data = recovery_routes_srv()
	except rospy.ServiceException, e:
		log_message("\t\tService call failed: %s"%e)
		return -1
	# if the radian angle == -1.0, that indicates there are none.  return failure
	if data.rad_angle == -1.0:
		log_message('No recovery angles')
		return -1
	# otherwise, convert the radian angle into degrees
	else:
		[trans, rot] = get_robot_position()
		relative_angle   = (data.rad_angle - rot[2]) % (2*math.pi)
		print 'recovery_angle_rad:', data.rad_angle
		print 'rot[2]:            ', rot[2]
		print 'relative_angle:    ', relative_angle
		print 'recovery_angle_deg:', math.degrees(relative_angle)
		return math.degrees(relative_angle)

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
# check if robot is stuck
# prev_pos_time = [trans, rad_angle, time]
#########################################################
def robot_stuck(prev_pos_time, cur_pos_time):
	# refer to global variables
	global args, robot_is_stuck
	
	# if < args['robot_stuck_timeout'] seconds has elapsed, the robot is not stuck
	if (cur_pos_time[2] - prev_pos_time[2]) < args['robot_stuck_timeout']:
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
	global monitor_if_stuck, reset_stuck_monitor

	# initialize robot position and time
	cur_time = time.time()
	[trans, rot] = get_robot_position()
	prev_pos_time = [trans, rot[2], cur_time]

	while True:
		if not monitor_if_stuck or reset_stuck_monitor:
			# reset robot position and time
			cur_time = time.time()
			[trans, rot] = get_robot_position()
			prev_pos_time = [trans, rot[2], cur_time]
			reset_stuck_monitor = False
		else:
			# get the current time and position of robot 
			cur_time = time.time()
			[trans, rot] = get_robot_position()
			cur_pos_time = [trans, rot[2], cur_time]
			# check if the robot is stuck
			prev_pos_time = robot_stuck(prev_pos_time, cur_pos_time)

		time.sleep(0.5)

#########################################################
# set the fsm state
#########################################################
def set_fsm_state(old, new):
	if not old == new:
		log_message('fsm_state: ' + str(new))
		old = new
	return old, new

#########################################################
# set the fsm status
#########################################################
def set_fsm_status(old, new):
	if not old == new:
		log_message(new)
		narration_pub.publish(new)
		old = new
	return old, new

#########################################################
# wander
#
# The following code is modeled as a FSM
# State						Signal					Action			New State
# (*) any					command=stop			stop			standby
# (1) make_decision			dir =  F, !L, !R, ?TA	---				drive_forward
#							dir = !F, !L, !R, TA	issue_ta		turn_around
#							dir = ?F,  L,  R, ?TA	issue_rotate	rotate
#													---				forward_through_int
#							dir = !F, !L, !R, !TA	report_failure	rotate_recovery
# (2) rotate				done_rotating			---				forward_through_int
# 							otherwise				---				rotating
# (3) forward_through_int	intersec changes		---				forward
#							at_dead_end				---				make_decision
#							otherwise				issue_forward	forward_through_int
# (4) drive_forward			at_intersection			stop			make_decision
#							at_dead_end				---				make_decision
# 							otherwise				issue_forward	drive_forward
# (5) rotate_recovery       done_rotating			---				make_decision
#
#########################################################
def wander():
	# refer to global objects
   	global command, monitor_if_stuck, reset_stuck_monitor, internal_status, routes, move_base_goal_status, robot_is_stuck

	# initialize local variables
	fsm_state                  = 1
	old_fsm_state              = None
	fsm_status                 = ''
	last_fsm_status            = None
	last_internal_status       = None
	cur_goal                   = None
	cur_intersection           = None
	start_position, rot        = get_robot_position()
	num_fwd_updates            = 0
	rotating_to_recovery_angle = False
	new_direction              = ''

	# loop forever
	while True:
		# service commands
		if command == 'stop' and not internal_status == 'standby':
			movebase_stop()
			internal_status  = 'standby'
			monitor_if_stuck = False
		if command == 'start' and internal_status == 'standby':
			narration_pub.publish('WANDER')
			internal_status            = 'working_on_it'
			fsm_state                  = 1 
			old_fsm_state              = None
			fsm_status                 = ''
			last_fsm_status            = None
			cur_goal                   = None
			cur_intersection           = None
			start_position, rot        = get_robot_position()
			num_fwd_updates            = 0
			robot_is_stuck             = False
			monitor_if_stuck           = True
			rotating_to_recovery_angle = False
			new_direction              = ''

		# publish status
		if not internal_status == last_internal_status:
			log_message('internal_status: ' + internal_status)
			status_pub.publish(internal_status)
			last_internal_status = internal_status
		
		# if internal status is anything besides 'working_on_it', sleep 
		if not internal_status == 'working_on_it':
			time.sleep(0.3)
			continue
	
		# Check if stuck
		#################################
		if robot_is_stuck:
			log_message('robot is stuck!')
			movebase_stop()
			old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 1)
			cur_goal = None
			robot_is_stuck = False
			reset_stuck_monitor = True
			num_fwd_updates = 0
		
		# STATE 0 (standby)
		#################################
		if fsm_state == 0:
			last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'Standby.')

		# STATE 1 (eval_cmd)
		#################################
		elif fsm_state == 1:
			last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'Deciding what to do next.')
			possible_directions     = get_possible_directions()
			intersection_directions = get_intersection_dirs()
			log_message('possible_directions: ' + ','.join(possible_directions)) 
			log_message('intersection_directions: ' + ','.join(intersection_directions)) 
			
			# if not at a predefined intersection (or an inactive intersection) but can go forward, go forward
			if (routes['intersection'] == 'undefined' or len(intersection_directions) == 0) and 'forward' in possible_directions:
				num_fwd_updates = 0
				old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 4)
			# we are at a predefined intersection: choose which way to go
			elif not routes['intersection'] == 'undefined' and len(intersection_directions) > 0:
				new_direction = get_direction_to_go()
				#_, new_direction = get_random_direction()
				log_message('choosing to go ' + new_direction)
				if new_direction == 'forward':
					cur_intersection = routes['intersection']
					old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 3)
					start_position, _ = get_robot_position()
				else:
					rotate(new_direction)
					old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 2)
			# some other directions available
			else:
				old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 5)

		# STATE 2(rotate)
		#################################
		elif fsm_state == 2:
			if rotating_to_recovery_angle:
				last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'Rotating to recovery angle.')
			else:
				last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'Rotating ' + new_direction + '.')
			if move_base_goal_status > 2:
				num_fwd_updates = 0
				start_position, _ = get_robot_position()
				if rotating_to_recovery_angle:
					rotating_to_recovery_angle = False
					old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 4)
				else:
					old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 3)
					cur_intersection = routes['intersection']

		# STATE 3 (forward_through_int)
		#################################
		elif fsm_state == 3:
			last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'driving through intersection')
			cur_position, rot = get_robot_position()
			if (routes['intersection'] == 'undefined' or not cur_intersection == routes['intersection']) and dist(start_position, cur_position) > 2.0:
				old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 4)
			else:
				# update our forward goal
				cur_goal, num_fwd_updates = update_forward_goal(cur_goal, rot[2], num_fwd_updates)
				# if we have cannot drive forward anymore, we have reached a dead end
				# without driving through this intersection
				if move_base_goal_status > 2:
					last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'failure driving through intersection')
					movebase_stop()
					old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 1)
					cur_goal = None

		# STATE 4 (drive_forward)
		#################################
		elif fsm_state == 4:
			last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'driving forward')
			intersection_directions = get_intersection_dirs()
			#log_message('intersection_directions: ' + ','.join(intersection_directions)) 
			#possible_directions = get_possible_directions()
			#log_message('possible_directions: ' + ','.join(possible_directions)) 
			
			have_left_or_right_turn = ('left' in intersection_directions or 'right' in intersection_directions)
			if routes['intersection'] in ['elbow', 'three-way', 'four-way'] and have_left_or_right_turn:
				movebase_stop()
				cur_goal = None
				old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 1)
			else:
				# compute the angle that constitutes forward
				[trans, rot] = get_robot_position()
				# if the robot just started driving forward, use the robot's current pose
				if dist(start_position, trans) < 2.0:
					forward_angle = rot[2]
				# compute the angle from the start_position to the robot's current position
				else:
					circle_x = trans[0] - start_position[0]
					circle_y = trans[1] - start_position[1]
					forward_angle = math.atan2(circle_y, circle_x)
				# update our forward goal
				cur_goal, num_fwd_updates = update_forward_goal(cur_goal, forward_angle, num_fwd_updates)
				# if we have cannot drive forward anymore, we have reached a dead end
				if move_base_goal_status > 2:
					last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'At end of hall.  Performing dead-end rotate recovery.')
					rotate_recovery()
					cur_goal = None
					old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 1)

		# STATE 5 (rotate_recovery)
		#################################
		elif fsm_state == 5:
			# first check if there is any direction we can rotate in
			possible_directions = get_possible_directions()
			recovery_angle      = get_recovery_direction()
			if recovery_angle == -1 or len(possible_directions) == 0:
				last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'Rotate Recovery.')
				rotate_recovery()
				old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 1)
			else:
				last_fsm_status, fsm_status = set_fsm_status(last_fsm_status, 'Recovery rotate angle ' + str(recovery_angle) + '.')
				rotate(recovery_angle)
				rotating_to_recovery_angle = True
				old_fsm_state, fsm_state = set_fsm_state(old_fsm_state, 2)

#########################################################
# update all routes
#########################################################
def update_routes(data):
	# refer to global objects
   	global routes, fresh_routes

	# load this data structure:
	# routes['intersection']             = intersection_name
	# routes['intersection_id']          = intersection_id
	# routes['center']                   = [x_map, y_map]
	# routes['new_intersection']         = True/False
	# routes['paths'][path_label]        = [path_active, direction, x, y]
	# routes['dirs'][dir_name][path_len] = [x_map, y_map]
	# routes['entry_path_label']         = entry_path_label
	routes = json.loads(data.data)
	fresh_routes = True

##############################################
# Update command (can be 'start', 'stop')
# Status can be 'working_on_it', 'standby')
# if 'start', return True once the code has started working on it
# if 'stop', return True once the code has completely stopped
##############################################
def command_callback(data):
	# refer to global objects
	global command, internal_status

	# update command
	command = data.command
	log_message("Got new command: " + command)

	# loop until the code has done the appropriate thing
	while True:
		if command == 'start' and not internal_status == 'standby':
			clear_mouth_buf_pub.publish(True)
			return commandNodeResponse(True)
		if command == 'stop' and internal_status == 'standby':
			return commandNodeResponse(True)
		time.sleep(0.1)

######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	print msg
	args['output_log'].write(datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + ' ' + msg + '\n')
	
#########################################################
# if calling from the command line...
#########################################################
if __name__ == '__main__':
	# import launch file params 
	args = {}
	args['goal_update_dist']    = rospy.get_param('wander/goal_update_dist')
	args['robot_stuck_timeout'] = rospy.get_param('wander/robot_stuck_timeout')

	# open output log
	dateTimeObj        = datetime.now()
	output_filename    = dateTimeObj.strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
	home               = os.path.expanduser("~")
	log_path           = home + '/.ros/wander/log/'
	if not os.path.exists(log_path):
		os.makedirs(log_path)
	output_log         = open(log_path + output_filename, 'w')
	args['output_log'] = output_log
	log_message('Starting wander')

	# initialize global variables
	command                    = 'stop'
	internal_status            = 'standby'
	move_base_goal_status      = -2 # indicates 'no goals'
	routes                     = {'intersection': 'undefined', 'paths': {}, 'dirs': {}}
	fresh_routes               = False
	new_goal_active            = False
	last_move_base_goal_status = None
	robot_is_stuck             = False
	monitor_if_stuck           = False
	reset_stuck_monitor        = False
	int_paths_taken            = {}

	try:
		# setup rviz marker publisher
		rviz_publisher = rviz_marker_publisher.setup()

		# initialize this node
		rospy.init_node('wander')

		# get the transform
		tf_ = tf.TransformListener()

		# setup subscriptions
		rospy.Subscriber("/intersection_mapper/routes", Routes2, update_routes)
		rospy.Subscriber("/move_base/status", GoalStatusArray, move_base_callback, queue_size=1)
		rospy.Subscriber('/wander/start_monitor_stuck', Bool, monitor_robot_stuck)

		# setup publishers
		status_pub          = rospy.Publisher("/wander/status", String, queue_size=1)
		narration_pub       = rospy.Publisher('/narration',     String, queue_size=1)
		cmd_vel_pub         = rospy.Publisher('/cmd_vel',       Twist,  queue_size=1)
		monitor_stuck_pub   = rospy.Publisher('/wander/start_monitor_stuck',  Bool, queue_size=1)
		clear_mouth_buf_pub = rospy.Publisher("/mouth_and_ears/clear_say_buffer", Bool, queue_size=1)

		# setup services
		rospy.Service('/wander/command', commandNode, command_callback)

		# wait one second, then set flag to start monitoring for robot being stuck
		time.sleep(1)
		monitor_stuck_pub.publish(True)

		# loop forever
		wander()
	except rospy.ROSInterruptException:
		pass

