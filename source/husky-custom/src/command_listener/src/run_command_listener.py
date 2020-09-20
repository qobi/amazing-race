#!/usr/bin/env python

#########################################################
# import libraries
#########################################################
import os
import re
import sys
import rospy
import string
from datetime import datetime
from std_msgs.msg import String

######################################################################################
# check whether the user is requesting an easter egg response
######################################################################################
def check_easter_egg(phrase):
	if 'name' in phrase:
		mouth_pub.publish('My name is hosh.')
	elif 'how are you' in phrase:
		mouth_pub.publish('Great.  I am ready to find something.  I would wag my tail right now if I had one.')
	elif 'smart' in phrase and 'ai' in phrase and 'researcher' in phrase:
		mouth_pub.publish('The smartest AI research is Professor Jeffrey Siskind from Purdue.')
	elif 'favorite' in phrase and 'student' in phrase:
		mouth_pub.publish('My favorite grad student is Jared.')
	elif 'you' in phrase and 'intelligent' in phrase:
		mouth_pub.publish('I am of average intelligence.  I am smarter than some humans.  But not smarter than my creators.')
	elif 'best' in phrase and 'university' in phrase:
		mouth_pub.publish('The best university is Purdue.')
	else:
		mouth_pub.publish('I did not understand that.')


def get_room_number(desired_destination):
	room_found = room_regex.findall(' ' + desired_destination.replace(' ', '  ') + ' ')
	if len(room_found) > 0:
		words = desired_destination.split()
		room_found = room_found[0].strip()
		room_idx = words.index(room_found)
		if room_idx < len(words)-1 and len(words[room_idx+1]) == 1:
			room_found += words[room_idx+1]
		return room_found.strip()
	else:
		return None

######################################################################################
# execute command
######################################################################################
def execute_command(utterance, written_command=False):
	# strip white space (and convert to lowercase)
	utterance = utterance.strip().lower()
	# remove punctuation
	utterance = utterance.replace(',', '').replace('.', '')

	# save command issued
	if written_command:
		log_message('Written: ' + utterance)
	else:
		log_message('Spoken: ' + utterance)
	
	# check to see if the heard_speech is a command to carry out
	if utterance.split()[0] == args['wake_word']:
		narration_pub.publish('Command Listener: Received Command')
		if not written_command:
			print ''
		output_msg = 'received command:' + ' '.join(utterance.split()[1:])
		log_message(output_msg)

		if not written_command:
			print "command to issue: "
		# determine whether the command is a task to execute...or just an easter egg
		if 'stop' in utterance:
			command_pub.publish('stop')
			narration_pub.publish('Command Listener: Stop everything.')
			mouth_pub.publish("stopping")
		elif 'find' in utterance:
			exclude = set(string.punctuation)
			utterance = ''.join(ch for ch in utterance if ch not in exclude).lower()

			words = utterance.split()
			find_idx = 0
			for idx, word in enumerate(words):
				if 'find' in word:
					find_idx = idx
					break
			desired_destination = ' '.join(words[find_idx+1:]).strip()
			room_number = get_room_number(desired_destination)
			if room_number == None:
				mouth_pub.publish("I did not understand that goal.")
			else:
				command_pub.publish('find ' + room_number)
				narration_pub.publish('Command Listener: Find ' + room_number + '.')
				if args['debug']: mouth_pub.publish("I will find " + room_number)
				else:             mouth_pub.publish("OK.")
		elif 'arrived' in utterance:
			new_location = utterance.split('at')[-1].strip()
			command_pub.publish('new_loc ' + new_location)
			narration_pub.publish('Command Listener: At new location "' + new_location + '".')
		elif 'wander' in utterance or 'wonder' in utterance:
			command_pub.publish('wander')
		elif 'elevator' in utterance and ('no' in utterance or 'not' in utterance):
			command_pub.publish('elevator_error')
		elif 'skywalk' in utterance and ('no' in utterance or 'not' in utterance):
			command_pub.publish('skywalk_error')
		else:
			check_easter_egg(utterance)

######################################################################################
# process spoken command
######################################################################################
def process_spoken_command(data):
	utterance = data.data
	execute_command(utterance, False)

######################################################################################
# process written command
######################################################################################
def process_written_command():
	print "command to issue: ", 
	while True:
		utterance = raw_input()
		execute_command(args['wake_word'] + ' ' + utterance, True)
		print "command to issue: ", 

######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	print msg
	args['output_log'].write(datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + ' ' + msg + '\n')
	
######################################################################################
# when starting up...
######################################################################################
if __name__ == '__main__':
	# import launch file params
	args = {}
	args['wake_word'] = rospy.get_param('command_listener/wake_word').lower()
	args['debug']     = rospy.get_param('command_listener/debug')

	# open output log
	dateTimeObj        = datetime.now()
	output_filename    = dateTimeObj.strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
	home               = os.path.expanduser("~")
	log_path           = home + '/.ros/command_listener/log/'
	if not os.path.exists(log_path):
		os.makedirs(log_path)
	output_log         = open(log_path + output_filename, 'w')
	args['output_log'] = output_log
	log_message('Starting command listener')

	# compile regex for rooms
	door_regex_str = '\s[a-zA-Z]{0,1}[0-9]{1,4}[a-zA-Z]{0,1}\s'
	room_regex = re.compile(door_regex_str)

	try:
		# initialize node
		rospy.init_node('command_listener')

		# setup subscribers
		rospy.Subscriber('/mouth_and_ears/heard_speech', String, process_spoken_command)

		# setup publishers
		command_pub   = rospy.Publisher('/command_listener/command', String, queue_size=1)
		mouth_pub     = rospy.Publisher('/mouth_and_ears/say',       String, queue_size=1)
		narration_pub = rospy.Publisher('/narration',                String, queue_size=1)
		
		# loop forever
		#process_written_command()
		rospy.spin()
	except rospy.ROSInterruptException: 
		pass


'''
import re
import string

# compile regex for rooms
door_regex_str = '\s[a-zA-Z]{0,1}[0-9]{1,3}[a-zA-Z]{0,1}\s'
room_regex = re.compile(door_regex_str)

def get_room_number(desired_destination):
	room_found = room_regex.findall(' ' + desired_destination.replace(' ', '  ') + ' ')
	if len(room_found) > 0:
		words = desired_destination.split()
		room_found = room_found[0].strip()
		room_idx = words.index(room_found)
		if room_idx < len(words)-1 and len(words[room_idx+1]) == 1:
			room_found += words[room_idx+1]
		return room_found.strip()
	else:
		return None

def get_goal(utterance):
	exclude = set(string.punctuation)
	utterance = ''.join(ch for ch in utterance if ch not in exclude).lower()
	words = utterance.split()
	find_idx = 0
	for idx, word in enumerate(words):
		if 'find' in word:
			find_idx = idx
			break
	desired_destination = ' '.join(words[find_idx+1:]).strip()
	room_number = get_room_number(desired_destination)
	print room_number


get_goal('Find 235')
get_goal('Go find room 240a.')
get_goal('I want you to find room 313 G.')
get_goal('Find B12')
get_goal("hosh can you get mary ann's office?")

'''





