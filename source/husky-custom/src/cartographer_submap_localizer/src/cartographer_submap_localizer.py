#!/usr/bin/env python

#########################################################
# import libraries
#########################################################
import tf
import sys
import time
import math
import rospy
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/robot-slang/workspaces/cartographer-ws/install_isolated/share/')
#from cartographer_ros_msgs.msg import *
from cartographer_ros_msgs.msg import SubmapList

# global variables
latest_submap = []
all_submaps   = []

#########################################################
# convert a global point to its relative submap point
# this is needed to localize point after global slam
# shifts map (often as a result of loop closure) 
#########################################################
def get_submap_point(global_point, specific_trajectory_id=None, specific_submap_index=None):
	# refer to global objects
	global latest_submap, all_submaps

	# if the user didn't specify a specific trajectory_id and submap_index, use the latest 
	submap = latest_submap

	# otherwise find the appropriate submap
	if not (specific_trajectory_id == None and specific_submap_index == None):
		for this_submap in all_submaps:
			if this_submap.trajectory_id == specific_trajectory_id and this_submap.submap_index == specific_submap_index:
				submap = this_submap
	
	# get details about the submap
	trajectory_id = submap.trajectory_id
	submap_index  = submap.submap_index
	orientation   = submap.pose.orientation

	# get point relative to the submap
	rel_x = global_point[0] - submap.pose.position.x
	rel_y = global_point[1] - submap.pose.position.y

	# return submap point data
	submap_point  = (trajectory_id, submap_index, orientation, rel_x, rel_y)
	return submap_point

#########################################################
# convert a submap point to its absolute global point
# using the latest submaps.  This gives an accurate point
# of the object in the latest and greatest map (including
# if the map has shifted as a result of loop closure). 
#########################################################
def convert_submap_point_to_global_point(submap_point):
	# refer to global objects
	global all_submaps

	# extract submap point data
	trajectory_id, submap_index, orientation, rel_x, rel_y = submap_point

	# find the appropriate submap
	for submap in all_submaps:
		if not submap.trajectory_id == trajectory_id:
			continue
		if not submap.submap_index == submap_index:
			continue

		# if the submap orientation has changed, compute the difference
		if not submap.pose.orientation == orientation:
			# get the current orientation
			quaternion = [submap.pose.orientation.x, submap.pose.orientation.y, submap.pose.orientation.z, submap.pose.orientation.w]
			cur_submap_rot = tf.transformations.euler_from_quaternion(quaternion)

			# get the original orientation
			quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
			orig_submap_rot = tf.transformations.euler_from_quaternion(quaternion)

			# compute the relative rotation
			rel_rot = cur_submap_rot[2] - orig_submap_rot[2]

			# compute the polar coordinates of the point on the submap
			theta = math.atan2(rel_y, rel_x)
			mag = math.sqrt(rel_x*rel_x + rel_y*rel_y)

			# take into account the rotation of the submap
			rotated_theta = theta + rel_rot

			# compute the location of the point on the submap
			rel_x = math.cos(rotated_theta) * mag
			rel_y = math.sin(rotated_theta) * mag

		# compute the location of the point on the global map
		abs_x = submap.pose.position.x + rel_x
		abs_y = submap.pose.position.y + rel_y

		return [abs_x, abs_y]

#########################################################
# save off the latest and greatest submap
#	trajectory_id: 0
#	submap_index: 19
#	submap_version: 1
#	pose: 
#	  position: 
#		x: 6.88570672598
#		y: 22.9500486011
#		z: 0.0
#	  orientation: 
#		x: 0.0
#		y: 0.0
#		z: 0.00648435383753
#		w: 0.999978976357
#	is_frozen: False
#########################################################
def submap_callback(data):
	global latest_submap, all_submaps
	all_submaps = data.submap
	if len(data.submap) > 0:
		latest_submap = data.submap[-1]

'''
	print 'saving pickles'
	import pickle
	with open('latest_submap.pickle', 'wb') as handle:
		pickle.dump(latest_submap, handle, protocol=pickle.HIGHEST_PROTOCOL)
	with open('all_submaps.pickle', 'wb') as handle:
		pickle.dump(all_submaps, handle, protocol=pickle.HIGHEST_PROTOCOL)
	print 'done saving pickles'
'''
def temp():
	global latest_submap, all_submaps

	import pickle
	print 'loading latest'
	with open('/home/jjohanse/robot-slang/husky-custom/src/cartographer_submap_localizer/src/latest_submap.pickle', 'rb') as handle:
		latest_submap = pickle.load(handle)
	with open('/home/jjohanse/robot-slang/husky-custom/src/cartographer_submap_localizer/src/all_submaps.pickle', 'rb') as handle:
		all_submaps = pickle.load(handle)

######################################################################################
# initialize variables/subscribers 
######################################################################################
def setup():
	# get the transform
	tf_ = tf.TransformListener()

	# setup subscriptions
	rospy.Subscriber('/submap_list', SubmapList, submap_callback, queue_size=1)
	rospy.wait_for_message('/submap_list', SubmapList)

'''
rosparam set use_sim_time true
rosbag play [bag name].bag --clock
'''

