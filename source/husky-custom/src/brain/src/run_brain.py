#!/usr/bin/env python

#########################################################
# import libraries
#########################################################
import os 
import tf
import sys
import time
import rospy
import wander
import actionlib
from datetime import datetime
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from wander.srv import commandNode as wander_command
from approach_person.srv import commandNode as approach_person_command
from hold_conversation.srv import conversationContent
from hold_conversation.srv import commandNode as conversation_command
from follow_directions.srv import directionsContent
from follow_directions.srv import commandNode as follow_directions_command
from door_navigation.srv import doorTag as door_navigation_command
from intersection_mapper.msg import Routes
from std_msgs.msg import String, Bool

#########################################################
# update statuses
#########################################################
def update_wander_status(data):
	global wander_status
	if not data.data == wander_status:
		log_message('wander status: ' + data.data)
	wander_status = data.data

def update_app_person_in_view_status(data):
	global approachable_person_in_view
	approachable_person_in_view = data.data

def update_approach_person_status(data):
	global approach_person_status
	if not data.data == approach_person_status:
		log_message('approach_person status: ' + data.data)
	approach_person_status = data.data

def update_hold_conversation_status(data):
	global hold_conversation_status
	if not data.data == hold_conversation_status:
		log_message('hold_conversation status: ' + data.data)
	hold_conversation_status = data.data

def update_follow_directions_status(data):
	global follow_directions_status
	if not data.data == follow_directions_status:
		log_message('follow_directions status: ' + data.data)
	follow_directions_status = data.data

def update_door_navigation_status(data):
	global door_navigation_status
	if not data.data == door_navigation_status:
		log_message('door_navigation status: ' + data.data)
	door_navigation_status = data.data

def update_directions(data):
	global directions
	directions = data.data

#########################################################
# send start/stop commands to nodes
#########################################################
def wander_cmd(cmd):
	log_message('wander cmd: ' + cmd)
	# if we are wandering, we should be on the lookout for approachable people
	if cmd == 'start': approach_person_cmd('start-tracking')

	try:
		command_service = rospy.ServiceProxy('/wander/command', wander_command)
		return command_service(cmd)
	except rospy.ServiceException, e:
		log_message("\t\tService call failed: %s"%e)
		return False
	
def approach_person_cmd(cmd):
	log_message('approach_person cmd: ' + cmd)
	try:
		command_service = rospy.ServiceProxy('/approach_person/command', approach_person_command)
		return command_service(cmd)
	except rospy.ServiceException, e:
		log_message("\t\tService call failed: %s"%e)
		return False

def hold_conversation_cmd(cmd, location=''):
	log_message('hold_conversation cmd: ' + cmd)
	# if we are having a conversation, we do not need to be on the lookout for approachable people
	if cmd == 'start': approach_person_cmd('stop-tracking')

	if not location == '':
		try:
			conversation = rospy.ServiceProxy('/hold_conversation/conversationContent', conversationContent)
			conversation('get_directions', location)
		except rospy.ServiceException, e:
			log_message("\t\tService call failed: %s"%e)	
	try:
		command_service = rospy.ServiceProxy('/hold_conversation/command', conversation_command)
		return command_service(cmd)
	except rospy.ServiceException, e:
		log_message("\t\tService call failed: %s"%e)	
		return False

def follow_directions_cmd(cmd, directions=''):
	log_message('follow_directions cmd: ' + cmd)
	if not directions == '':
		try:
			publish_directions = rospy.ServiceProxy('/follow_directions/directions', directionsContent)
			publish_directions(directions)
		except rospy.ServiceException, e:
			log_message("\t\tService call failed: %s"%e)	
	try:
		command_service = rospy.ServiceProxy('/follow_directions/command', follow_directions_command)
		return command_service(cmd)
	except rospy.ServiceException, e:
		log_message("\t\tService call failed: %s"%e)	
		return False

def door_navigation_cmd(cmd, door_tag='', door_dir='all', door_index=0):
	log_message('door_navigate cmd: ' + cmd)
	try:
		if   door_dir == 'goal-L': door_dir = 'left'
		elif door_dir == 'goal-R': door_dir = 'right'
		else:                      door_dir = 'all'
		command_service = rospy.ServiceProxy('/door_navigation/command', door_navigation_command)
		return command_service(cmd, door_tag, door_dir, door_index)
	except rospy.ServiceException, e:
		log_message("\t\tService call failed: %s"%e)	
		return False

#########################################################
# find a location
# This function acts as a multi-FSM.  Below is a description of 
# the states and the signals that lead to other states
#########################################################
def run_fsm():
	# refer to global variables
   	global approachable_person_in_view, approach_person_status, hold_conversation_status, follow_directions_status, \
   		   door_navigation_status, directions, command, state, location

	# run the FSM
	while True:
		# execute commands
		if command == 'stop' and not state == 'standby':
			wander_cmd('stop')
			approach_person_cmd('stop')
			hold_conversation_cmd('stop')
			follow_directions_cmd('stop')
			approach_person_cmd('stop-tracking')
			state = 'standby'

		###########################
		# WANDER FSM
		# 
		# (-) standby			command=wander					wander
		# (0) wander			status=failure					standby
		#						otherwise						wander
		###########################
		# STATE 0: wander
		if command == 'wander' and state == 'standby':
		   	state = 'wander'
		   	wander_cmd('start')
		if command == 'wander' and state == 'wander' and wander_status == 'failure':
			wander_cmd('stop')
			state = 'standby'

		###########################
		# FIND FSM
		#
		# State					Signal							New State
		# (*) any				command=stop					standby
		# (-) standby			command=find					wander
		# (0) wander			approachable_person_in_view		approach_person
		#						otherwise						wander
		# (1) approach_person	status=failure					wander
		#						status=complete					hold_conversation
		#						otherwise						approach_person					
		# (2) hold_conversation status=complete & directions	follow_directions
		#						status=complete & !directions	wander
		#						otherwise						hold_conversation
		# (3) follow_directions	status=failure					wander
		#						status=complete					door_navigation
		#						otherwise						follow_directions
		# (4) door_navigation	status=failure					wander
		#						status=complete					goal
		#						otherwise						door_navigatio
		# (5) goal				None							standby
		###########################
		# STATE 0: wander
		if command == 'find' and state == 'standby':
			wander_cmd('start')
		   	state = 'wander'
		if command == 'find' and state == 'wander' and approachable_person_in_view:
			wander_cmd('stop')
			approach_person_cmd('start')
			state = 'approach_person'

		# STATE 1: approach_person
		if command == 'find' and state == 'approach_person' and not approach_person_status == 'working_on_it':
			if approach_person_status == 'complete':
				approach_person_cmd('stop')
				hold_conversation_cmd('start', location)
				state = 'hold_conversation'
			else:
				approach_person_cmd('stop')
				wander_cmd('start')
				state = 'wander'

		# STATE 2: hold_conversation
		if command == 'find' and state == 'hold_conversation' and not hold_conversation_status == 'working_on_it':
			if directions == '' or directions == 'person':
				hold_conversation_cmd('stop')
				# tell the robot to robot 90 degrees before resuming wandering
				follow_directions_cmd('start', 'right,person')
				#wander_cmd('start')
				state = 'follow_directions'
			else:
				hold_conversation_cmd('stop')
				follow_directions_cmd('start', directions)
				state = 'follow_directions'

		# STATE 3: follow_directions
		if command == 'find' and state == 'follow_directions' and follow_directions_status == 'failure':
			follow_directions_cmd('stop')
			wander_cmd('start')
			state = 'wander'
		if command == 'find' and state == 'follow_directions' and follow_directions_status == 'complete':
			follow_directions_cmd('stop')
			if directions.split(',')[-1] == 'person':
				wander_cmd('start')
				state = 'wander'
			else:
				door_navigation_cmd('start', location, directions.split(",")[-1])
				state = 'door_navigation'

		# STATE 4: door_navigation
		if command == 'find' and state == 'door_navigation' and door_navigation_status == 'failure':
			door_navigation_cmd('stop')
			wander_cmd('start')
			state = 'wander'
		if command == 'find' and state == 'door_navigation' and door_navigation_status == 'complete':
			door_navigation_cmd('stop')
			state = 'goal'
		
		# STATE 5: goal
		if command == 'find' and state == 'goal':
			narration_pub.publish('ACHIEVE GOAL')
			log_message('arrived at goal!')
			mouth_pub.publish("I have arrived at " + location)
			state = 'standby'
			command = 'stop'
		
		# wait some time before looping
		rospy.sleep(0.5)

#########################################################
# execute command given by human
#########################################################
def execute_command(data):
	# refer to global variables
   	global command, location

	# get the command and arguments
	log_message('command: "' + data.data.split()[0] + '"')

	if data.data.split()[0] == "find":
		location = ' '.join(data.data.split()[1:])
		if args['debug']: mouth_pub.publish("I will find " + location)
		else:             mouth_pub.publish("OK.")
		command = 'find'
	elif data.data.split()[0] == "wander":
		mouth_pub.publish("I will wander")
		command = 'wander'
	elif data.data.split()[0] == "stop":
		mouth_pub.publish("stopping")
		command = 'stop'
	else:
		log_message("Invalid command.  Please send a valid command.")

######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	timestamp = datetime.now().strftime("%Y-%b-%d-%H-%M-%S")
	print timestamp + ' ' + msg
	args['output_log'].write(timestamp + ' ' + msg + '\n')

#########################################################
# if calling from the command line...
#########################################################
if __name__ == '__main__':
	# import launch file params 
	args = {}
	args['debug'] = rospy.get_param('brain/debug')

	# open output log
	dateTimeObj        = datetime.now()
	output_filename    = dateTimeObj.strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
	home               = os.path.expanduser("~")
	log_path           = home + '/.ros/brain/log/'
	if not os.path.exists(log_path):
		os.makedirs(log_path)
	output_log         = open(log_path + output_filename, 'w')
	args['output_log'] = output_log
	log_message('Starting brain')

	try:
		# initialize this node
		rospy.init_node('brain')

		# intialize global variables
		directions                  = ''
		location                    = ''
		command                     = 'stop'
		state                       = 'standby'
		wander_status               = 'standby'
		approach_person_status      = 'standby'
		hold_conversation_status    = 'standby'
		follow_directions_status    = 'standby'
		door_navigation_status      = 'standby'
		approachable_person_in_view = False

		# setup publisher
		mouth_pub      = rospy.Publisher('/mouth_and_ears/say', String, queue_size=1)
		narration_pub  = rospy.Publisher('/narration', String, queue_size=1)

		# setup subscriber
		rospy.Subscriber('/wander/status',                String, update_wander_status)
		rospy.Subscriber('/approach_person/status',       String, update_approach_person_status)
		rospy.Subscriber('/hold_conversation/status',     String, update_hold_conversation_status)
		rospy.Subscriber("/follow_directions/status",     String, update_follow_directions_status)
		rospy.Subscriber("/door_navigation/status",       String, update_door_navigation_status)
		rospy.Subscriber('/command_listener/command',     String, execute_command)
		rospy.Subscriber('/hold_conversation/result',     String, update_directions)
		rospy.Subscriber('/approach_person/app_person_in_view', Bool, update_app_person_in_view_status)


		# Create an action client called "move_base" with action definition file "MoveBaseAction"
		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

		# Waits until the action server has started up and started listening for goals.
		client.wait_for_server()

		# Creates a new goal with the MoveBaseGoal constructor
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.45
		goal.target_pose.pose.position.y = 0.0

		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = quaternion[2]
		goal.target_pose.pose.orientation.w = quaternion[3]

		# Sends the goal to the action server, then return control
		client.send_goal(goal)
		time.sleep(1.0)

		# run_fsm loops forever waiting for commands
		run_fsm()
	except rospy.ROSInterruptException:
		pass

# rostopic pub /command_listener/command std_msgs/String "find the bathroom" 

