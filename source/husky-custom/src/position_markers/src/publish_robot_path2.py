#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import tf
import math
import time
import pyttsx3
import sys
tf_ = tf.TransformListener()
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/robot-slang/husky-custom/src/rviz_marker_publisher/src/')
import rviz_marker_publisher
sys.path.append(home + '/robot-slang/husky-custom/src/cartographer_submap_localizer/src/')
import cartographer_submap_localizer as csl

#########################################################
# publish robot path in rviz
#########################################################
def publish_path(position_list):
	map_locations = []
	
	for idx, submap_point in enumerate(position_list):
		global_point = csl.convert_submap_point_to_global_point(submap_point)
		map_locations.append(global_point)	
	
	#print map_locations
	# keep track of rviz markers
	my_markers = []
	# delete existing markers
	marker_id = 1
	marker = {'id': marker_id, 'points':map_locations, 'scale': .2, 'color': [1.0, 0.0, 0.0]}
	line_marker = rviz_marker_publisher.create_line_strip_marker(marker)	
	my_markers.append(line_marker)
	# display all the sphere markers
	rviz_marker_publisher.display_markers(marker_pub, my_markers)

######################################################################################
# get robots position in the map frame
######################################################################################
def get_robot_position():
	# get robot's location
	now = rospy.Time.now()
	tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(2.0))
	trans, quad = tf_.lookupTransform( '/map', '/base_link', now )
	print (trans) # [3.5838475877987737, 0.7696075203471747, 0.0]
	submap_point = csl.get_submap_point(trans)
	return submap_point
	
if __name__== "__main__":
	rospy.init_node('position_markers')
	marker_pub  = rospy.Publisher('/position_markers', MarkerArray, queue_size=1)
	position_list = []
	csl.setup()
	while not rospy.is_shutdown():
		try:
			submap_point = get_robot_position()
			if not submap_point == None:
				position_list.append(submap_point)
			publish_path(position_list)
		except Exception, e:
			print 'error', e
			donothing=1
		time.sleep(0.2)


'''
rosparam set use_sim_time true
rosbag play [bag name].bag --clock

'''


