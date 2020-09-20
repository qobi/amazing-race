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
sys.path.append('/home/tilyevsk/robot-slang/husky-custom/src/rviz_marker_publisher/src/')
import rviz_marker_publisher
#########################################################
# publish doors in rvi
#########################################################
def publish_path(map_locations):
	# keep track of rviz markers
	my_markers = []
	# delete existing markers
	marker_id = 1
	marker = {'id': marker_id, 'points':map_locations, 'scale': .2, 'color': [1.0, 0.0, 0.0]}
	line_marker = rviz_marker_publisher.create_line_strip_marker(marker)	
	my_markers.append(line_marker)
	rviz_marker_publisher.display_markers(marker_pub, my_markers)

######################################################################################
# get robots position in the map frame
######################################################################################
def get_robot_position():
	# get robot's location
	now = rospy.Time.now()
	tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(2.0))
	(trans, quaternion) = tf_.lookupTransform( '/map', '/base_link', now )
	#(trans, quaternion) = tf_.lookupTransform( '/map', '/base_link', rospy.Time())
	print (trans)
	return trans
	
if __name__== "__main__":
	rospy.init_node('position_markers')
	marker_pub  = rospy.Publisher('/position_markers', MarkerArray, queue_size=1)
	position_list = []
	while not rospy.is_shutdown():
		try:
			position_list.append(get_robot_position())
			publish_path(position_list)
		except:
			donothing=1
		time.sleep(0.2)

