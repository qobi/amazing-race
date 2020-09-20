#!/usr/bin/env python

##############################################
# import libraries
##############################################
import os
import sys
import time
import rospy
import signal
import directions_conversation as dir_conv
from datetime import datetime
from hold_conversation.srv import *
from std_msgs.msg import String, Bool

#########################################################
# setup a signal handler to listen for exit signal (ctrl + c)
#########################################################
def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

##############################################
# update what the robot heard
##############################################
def update_heard_speech(data):
	# refer to global objects
	global response

	# replace numbers with text
	temp = data.data.lower()
	temp = temp.replace('1st', 'first')
	temp = temp.replace('2nd', 'second')
	temp = temp.replace('3rd', 'third')
	temp = temp.replace('4th', 'fourth')
	temp = temp.replace('5th', 'fifth')
	
	# update what the robot heard
	response = temp
	
##############################################
# update whether the human is speaking
##############################################
def update_human_speaking(data):
	# refer to global objects
	global human_speaking
	
	# update whether the human is speaking
	human_speaking = data.data

##############################################
# update whether the robot is speaking
##############################################
def update_robot_speaking(data):
	# refer to global objects
	global robot_speaking
	
	# update whether the robot is speaking
	robot_speaking = data.data

##############################################
# robot say something
##############################################
def say_robot(text):
	# refer to global objects
	global response, conv_exchange

	# save text spoken
	msg = 'robot: ' + text
	log_message(msg)
	conv_exchange.append(msg)

	# "say" the speech
	if args['conversation_medium'] == 'speech':
		mouth_pub.publish(text)

##############################################
# get human response
##############################################
def wait_for_human_response():
	# refer to global objects
	global response, args, human_speaking, robot_speaking

	# reset the response since this will be a new query
	response = ''

	if args['conversation_medium'] == 'text':
		response = raw_input("Human:  ")
	elif args['conversation_medium'] == 'speech':
		# wait for a response
		start_time = time.time()
		while True:
			# if the robot is speaking, clear out any response
			if robot_speaking:
				response = ''

			# check if we have a response
			if not response == '':
				return

			# if the human or robot is speaking, start timer over
			if human_speaking or robot_speaking:
				start_time = time.time()

			# if we have timed out listening for a response, return failure
			tot_time = time.time() - start_time
			if tot_time > args['listen_timeout']:
				return
			
			# wait a little bit before checking again
			rospy.sleep(0.2)
	else:
		log_message('Error!  Non-supported conversation medium: ' + args['conversation_medium'])

##############################################
# get directions by solving the constraint satisfaction problem
##############################################
def get_directions(X, prev_X, stuck, gave_instructions):
	# refer to global variables
	global response, internal_status, destination, conv_exchange

	# get all subsets of X with an unknown variable; generate a query for each subset
	subsets, X = dir_conv.generate_query(X, destination)
	if args['debug']:
		for subset in subsets: 
			log_message("\t" + str(subset['start_idx']) + ' ' + str(subset['end_idx']) + ' ' + subset['subset'] + ' ' + subset['query'])

	# use the first subset
	subset = subsets[0]
	say_robot(subset['query'])
	
	# wait for a response, then update X
	wait_for_human_response()
	msg = "Human: " + response
	log_message(msg)
	conv_exchange.append(msg)
	temp_X = list(X)
	X, message = dir_conv.update_X(prev_X, X, response, subset, destination)

	# perform inference
	X = dir_conv.perform_inference(args, X)

	# if the user does not know, bail
	if message == 'negated' and temp_X == [None]:
		X = ['person']
	# if the user negated the last interpretation, apologize and do so
	elif message == 'negated' and stuck < args['num_repeat']:
		say_robot('Sorry about that.  Let me ask my previous question.')
		time.sleep(1) # pause for the robot to start speaking

	# if the user told you to start over, do so
	if message == 'start-over':
		say_robot("Sorry about that.  Let's start over.")
		time.sleep(1) # pause for the robot to start speaking
		stuck = 0
	elif not temp_X == prev_X:
		prev_X = list(temp_X)
		stuck = 0

	# save off current state of plan
	output_msg = 'X: [' + ', '.join([str(x) for x in X]) + ']'
	log_message(output_msg)
	conv_exchange.append(output_msg)

	# keep track of how many times we've been stuck at this step
	if X == temp_X:
		stuck += 1
		if response == '':
			say_robot('I did not hear you.')
		else:
			if gave_instructions:
				say_robot('I did not understand that.')
			else:
				gave_instructions = True
				say_robot('I did not understand that.')
				say_robot('I understand verbal directions, like turn-around and turn left.  I understand intersections like an elbow or end of hallway.  I understand words like goal or destination.')
				say_robot('You can give me instructions in small chunks or all at once.  If I misunderstand something, you can tell me that my interpretation was wrong or incorrect and I will back up one step.  If I totally misunderstand, you can ask me to start over.')
		time.sleep(1) # pause for the robot to start speaking

	# output current plan
	output_msg = 'X: [' + ', '.join([str(x) for x in X]) + ']'
	log_message(output_msg)

	# check if we have a solution (i.e. X is complete and meets all constraints)
	if not None in X and dir_conv.all_constraints_satisfied(X):
		log_message('\tX is complete and meets all constraints!  We have a solution!')
		directions = ','.join(X)
		conv_result_pub.publish(directions)
		say_robot('Thanks for your help.  Have a great day!')
		if args['debug']: say_robot(dir_conv.summarize(X, destination))
		time.sleep(1) # pause for the robot to start speaking
		internal_status = 'complete'
	# X is complete but does not meets all constraints
	elif not None in X:
		log_message('\tError.  X is complete but does not meets all constraints!')
		conv_result_pub.publish('person')
		say_robot('Thanks for your help.  Have a great day!')
		time.sleep(1) # pause for the robot to start speaking
		internal_status = 'failure'
	# We have not progressed in our conversation; bail
	elif stuck > args['num_repeat']:
		log_message('\tError.  No progress is being made.  X is incomplete.  Keep the valid data!')
		valid_X = dir_conv.get_valid_X(X)
		if len(valid_X) < 2:
			conv_result_pub.publish('person')
			say_robot('I was not able to understand your directions.  Thanks anyway.  Good bye!')
			time.sleep(1) # pause for the robot to start speaking
			internal_status = 'failure'
		else:
			directions = ','.join(valid_X)
			conv_result_pub.publish(directions)
			say_robot('Thanks for your help.  Have a great day!')
			if args['debug']: say_robot(dir_conv.summarize(valid_X, destination))
			time.sleep(1) # pause for the robot to start speaking
			internal_status = 'complete'
	# X is not complete...so loop
	else:
		if args['debug']: log_message('\tLooping because X not complete.')

	# return data
	return X, prev_X, stuck, gave_instructions


##############################################
# save conversation log
##############################################
def save_conv_log():
	# refer to global variables
	global conv_filename, conv_exchange

	conv_log = open(conv_filename, 'w')
	for conv_line in conv_exchange:
		conv_log.write(conv_line + '\n')
	conv_log.close()
	conv_exchange = []

##############################################
# loop forever, starting/stopping conversation when commanded 
##############################################
def loop_forever():
	# refer to global variables
	global command, internal_status, robot_speaking, conv_filename, conv_exchange

	# initialize variables for get_directions conversation
	X      = [None]
	prev_X = [None]
	stuck  = 0
	gave_instructions = False

	# loop forever, holding various conversations when commanded 
	while True:
		# execute commands
		if command == 'stop' and not internal_status == 'standby':
			record_pub.publish(False)
			save_conv_log()
			internal_status = 'standby'
			log_message('internal_status: ' + internal_status)
		if command == 'start' and internal_status == 'standby':
			record_pub.publish(True)
			conv_filename =  conv_path + '/conv-' + datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + '.txt'
			narration_pub.publish('HOLD CONVERSATION')
			internal_status = 'working_on_it'
			log_message('internal_status: ' + internal_status)
			# reset variables for get_directions
			X      = [None]
			prev_X = [None]
			stuck  = 0
			gave_instructions = False

		# publish status
		status_pub.publish(internal_status)

		# if in any mode that is not 'working_on_it', loop 
		if not internal_status == 'working_on_it':
			time.sleep(0.2)
			continue

		# depending on the conversation type, take appropriate action
		if conversation_type == 'get_directions':
			X, prev_X, stuck, gave_instructions = get_directions(X, prev_X, stuck, gave_instructions)

		# continue to pause if the robot is still speaking
		while robot_speaking:
			do_nothing = 1

##############################################
# Update conversation params
##############################################
def update_conversation_params(data):
	# refer to global objects
	global conversation_type, destination, conv_topic, targetted_question

	# print conversation params
	log_message('New conversation parameters')
	log_message('\ttype:        ' + data.type)
	log_message('\tinitialInfo: ' + data.initialInfo)
	conversation_type = data.type

	# update conversation params according to conversation type
	if conversation_type == 'get_directions':
		destination = data.initialInfo
		return conversationContentResponse('params updated')
	elif conversation_type == 'targetted_question':
		conv_topic         = data.initialInfo.split('---')[0]	
		targetted_question = data.initialInfo.split('---')[1]
		return conversationContentResponse('params updated')
	else:
		return conversationContentResponse('unrecognized conversation type!')

##############################################
# Update command (can be 'start', 'stop')
# Status can be 'working_on_it', 'standby', 'complete', 'failure')
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
		if command == 'start' and internal_status == 'working_on_it':
			return commandNodeResponse(True)
		if command == 'stop' and internal_status == 'standby':
			return commandNodeResponse(True)
		time.sleep(0.1)

def audio_save():
    global frames
    waveFile = wave.open('/home/tilyevsk/wizard-of-oz/%s.wav'%(file_name), 'wb')
    waveFile.setnchannels(1)
    waveFile.setsampwidth(pyaudio.PyAudio().get_sample_size(pyaudio.paInt16))
    waveFile.setframerate(RATE)
    waveFile.writeframes(b''.join(frames))

######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	print msg
	args['output_log'].write(datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + ' ' + msg + '\n')

##############################################
# if calling from the command line
##############################################
if __name__ == '__main__':
	# import launch file params
	args = {}
	args['parser']              = rospy.get_param('hold_conversation/parser')
	args['models']              = rospy.get_param('hold_conversation/models')
	args['model_path']          = rospy.get_param('hold_conversation/model_path')
	args['conversation_medium'] = rospy.get_param('hold_conversation/conversation_medium')
	args['listen_timeout']      = rospy.get_param('hold_conversation/listen_timeout')
	args['num_instructions']    = rospy.get_param('hold_conversation/num_instructions')
	args['num_repeat']          = rospy.get_param('hold_conversation/num_repeat')
	args['debug']               = rospy.get_param('hold_conversation/debug')

	# open output log
	output_filename    = datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
	home               = os.path.expanduser("~")
	log_path           = home + '/.ros/hold_conversation/log/'
	conv_path          = home + '/.ros/hold_conversation/conv/'
	if not os.path.exists(log_path):
		os.makedirs(log_path)
	if not os.path.exists(conv_path):
		os.makedirs(conv_path)
	output_log         = open(log_path + '/' + output_filename, 'w')
	args['output_log'] = output_log
	log_message('Starting hold conversation')

	# initialize global variables
	command            = 'stop'
	internal_status    = 'standby'
	conv_filename      = conv_path + '/conv-init.txt'
	conv_exchange      = []
	response           = ''
	conversation_type  = ''
	destination        = ''
	conv_topic         = ''
	targetted_question = ''
	human_speaking     = False
	robot_speaking     = False

	try:
		# initialize node
		rospy.init_node('hold_conversation')

		# setup dir_conv
		dir_conv.setup(args)

		# setup services
		rospy.Service('/hold_conversation/conversationContent', conversationContent, update_conversation_params)
		rospy.Service('/hold_conversation/command', commandNode, command_callback)
		
		# setup subscribers
		rospy.Subscriber('/mouth_and_ears/heard_speech', String, update_heard_speech)
		rospy.Subscriber('/mouth_and_ears/human_speaking', Bool, update_human_speaking)
		rospy.Subscriber('/mouth_and_ears/robot_speaking', Bool, update_robot_speaking)

		# setup publishers
		mouth_pub       = rospy.Publisher('/mouth_and_ears/say', String, queue_size=1)
		record_pub      = rospy.Publisher('/mouth_and_ears/record', Bool, queue_size=1)
		status_pub      = rospy.Publisher('/hold_conversation/status', String, queue_size=1)
		conv_result_pub = rospy.Publisher('/hold_conversation/result', String, queue_size=1)
		narration_pub   = rospy.Publisher('/narration', String, queue_size=1)

		# loop forever
		loop_forever()
	except rospy.ROSInterruptException: 
		pass

# rosservice call /hold_conversation/conversationContent 'get_directions' 'the staircase'
# rosservice call /hold_conversation/command 'start'
# rosservice call /hold_conversation/command 'stop'

