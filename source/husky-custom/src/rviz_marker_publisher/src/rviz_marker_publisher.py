#!/usr/bin/env python

#############################################
# Code built from this sample:
# https://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/
#############################################

#############################################
# import libraries
#############################################
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import sys
import signal

#########################################################
# setup a signal handler to listen for exit signal (ctrl + c)
#########################################################
def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

#########################################################
# create arrow
# this_marker['id']		    = unique_id
# this_marker['x']		    = x
# this_marker['y']		    = y
# this_marker['quaternion'] = quaternion
# this_marker['scale']	    = [x, y, z] (float from 0 - 1)
# this_marker['color']	    = [R, G, B] (float from 0 - 1)
#########################################################
def create_arrow_marker(this_marker):
	# create arrow marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.ARROW
	marker.action = marker.ADD
	
	# Set the arrow features
	marker.scale.x = this_marker['scale'][0] # shaft diameter
	marker.scale.y = this_marker['scale'][1] # head diameter
	marker.scale.z = this_marker['scale'][2] # head length
	
	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.a = 1.0
	marker.color.r = this_marker['color'][0]
	marker.color.g = this_marker['color'][1]
	marker.color.b = this_marker['color'][2]
	
	# Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = this_marker['x']
	marker.pose.position.y = this_marker['y']
	marker.pose.position.z = 0.0
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = this_marker['quaternion'][2]
	marker.pose.orientation.w = this_marker['quaternion'][3]

	# set the marker ID
	marker.id = this_marker['id']

	# return arrow marker
	return marker

#########################################################
# create arrow
# this_marker['id']		    = unique_id
# this_marker['points']     = [[x1,y1], [x2,y2]]
# this_marker['scale']	    = [x, y, z] (float from 0 - 1)
# this_marker['color']	    = [R, G, B] (float from 0 - 1)
#########################################################
def create_arrow_marker_with_points(this_marker):
	# create arrow marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.ARROW
	marker.action = marker.ADD
	
	# Set the arrow features
	marker.scale.x = this_marker['scale'][0] # shaft diameter
	marker.scale.y = this_marker['scale'][1] # head diameter
	marker.scale.z = this_marker['scale'][2] # head length
	
	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.a = 1.0
	marker.color.r = this_marker['color'][0]
	marker.color.g = this_marker['color'][1]
	marker.color.b = this_marker['color'][2]
	
	# add the marker line points
	marker.points = []
	for idx in range(len(this_marker['points'])):
		new_point = Point()
		new_point.x = this_marker['points'][idx][0]
		new_point.y = this_marker['points'][idx][1]
		new_point.z = 0.0
		marker.points.append(new_point)

	# set the marker ID
	marker.id = this_marker['id']

	# return arrow marker
	return marker
#########################################################
# create sphere marker
# this_marker['id']    = unique_id
# this_marker['x']     = x
# this_marker['y']	   = y
# this_marker['scale'] = [x, y, z]
# this_marker['color'] = [R, G, B] (float from 0 - 1)
#########################################################
def create_sphere_marker(this_marker):
	# create sphere marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	
	# Set the scale of the marker; 0.2m radius; 1m tall
	marker.scale.x = this_marker['scale'][0]
	marker.scale.y = this_marker['scale'][1]
	marker.scale.z = this_marker['scale'][2]
	
	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.a = 1.0
	marker.color.r = this_marker['color'][0]
	marker.color.g = this_marker['color'][1]
	marker.color.b = this_marker['color'][2]
	
	# Set the pose of the marker.
	marker.pose.position.x = this_marker['x']
	marker.pose.position.y = this_marker['y']

	# set the marker ID
	marker.id = this_marker['id']

	return marker

#########################################################
# create cylinder marker
# this_marker['id']	   = unique_id
# this_marker['x']	   = x
# this_marker['y']	   = y
# this_marker['scale'] = [x, y, z]
# this_marker['color'] = [R, G, B] (float from 0 - 1)
#########################################################
def create_cylinder_marker(this_marker):
	# create cylinder marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.CYLINDER
	marker.action = marker.ADD
	
	# Set the scale of the marker; 0.2m radius; 1m tall
	marker.scale.x = this_marker['scale'][0]
	marker.scale.y = this_marker['scale'][1]
	marker.scale.z = this_marker['scale'][2]
	
	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.a = 1.0
	marker.color.r = this_marker['color'][0]
	marker.color.g = this_marker['color'][1]
	marker.color.b = this_marker['color'][2]
	
	# Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = this_marker['x']
	marker.pose.position.y = this_marker['y']
	marker.pose.position.z = 0.0

	# set the marker ID
	marker.id = this_marker['id']

	return marker

#########################################################
# create cube marker
# this_marker['id']    = unique_id
# this_marker['x']	   = x
# this_marker['y']	   = y
# this_marker['scale'] = [x, y, z]
# this_marker['color'] = [R, G, B] (float from 0 - 1)
# this_marker['alpha'] = a (float from 0 - 1)
#########################################################
def create_cube_marker(this_marker):
	# create cube marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.CUBE
	marker.action = marker.ADD
	
	# Set the scale of the marker
	marker.scale.x = this_marker['scale'][0]
	marker.scale.y = this_marker['scale'][1]
	marker.scale.z = this_marker['scale'][2]
	
	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.a = this_marker['alpha']
	marker.color.r = this_marker['color'][0]
	marker.color.g = this_marker['color'][1]
	marker.color.b = this_marker['color'][2]
	
	# Set the pose of the marker.
	marker.pose.position.x = this_marker['x']
	marker.pose.position.y = this_marker['y']

	# set the marker ID
	marker.id = this_marker['id']

	return marker

#########################################################
# create text marker
#########################################################
def create_text_marker(this_marker):
	# create text marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.TEXT_VIEW_FACING
	marker.action = marker.ADD
	
	# Set the scale of the marker; 0.2m radius; 1m tall
	marker.scale.z = this_marker['scale']
	
	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.a = 1.0
	marker.color.r = this_marker['color'][0]
	marker.color.g = this_marker['color'][1]
	marker.color.b = this_marker['color'][2]
	
	# Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = this_marker['x'] + 0.25
	marker.pose.position.y = this_marker['y'] + 0.25
	marker.pose.position.z = 0.0

	# set the text of the marker 
	marker.text = this_marker['name']

	# set the marker ID
	marker.id = this_marker['id']

	return marker

#########################################################
# create line_strip
# this_marker['id']	    = unique_id
# this_marker['points'] = [[x, y], [x, y], ...]
# this_marker['scale']  = scale
# this_marker['color']  = [R, G, B] (float from 0 - 1)
#########################################################
def create_line_strip_marker(this_marker):
	# create line marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.LINE_STRIP
	marker.action = marker.ADD
	
	# Set the scale of the marker; 0.2m radius; 1m tall
	marker.scale.x = this_marker['scale']
	
	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.a = 1.0
	marker.color.r = this_marker['color'][0]
	marker.color.g = this_marker['color'][1]
	marker.color.b = this_marker['color'][2]

	# add the marker line points
	marker.points = []
	for idx in range(len(this_marker['points'])):
		new_point = Point()
		new_point.x = this_marker['points'][idx][0]
		new_point.y = this_marker['points'][idx][1]
		new_point.z = 0.0
		if len(this_marker['points'][idx]) == 3:
			new_point.z = this_marker['points'][idx][2]
		marker.points.append(new_point)

	# set the marker ID
	marker.id = this_marker['id']

	return marker

#########################################################
# create delete_marker
#########################################################
def create_delete_marker(marker_id):
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.action = marker.DELETE
	marker.id = marker_id
	return marker

#########################################################
# create deleteall marker
#########################################################
def create_deleteall_marker():
	# create deleteall marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.action = marker.DELETEALL
	return marker
	
#############################################
# display_markers
# markers[idx] = marker
#############################################
def display_markers(publisher, my_markers):
	# create markerArray
	markerArray = MarkerArray()

	# loop over the markers and add them to the markerArray
	for this_marker in my_markers:
		markerArray.markers.append(this_marker)

	# Publish the MarkerArray
	publisher.publish(markerArray)

#############################################
# create_and_display_semantic_map_markers
# markers[idx]['id']   = unique_id
# markers[idx]['x']	   = x
# markers[idx]['y']	   = y
# markers[idx]['name'] = name of point
#############################################
def create_and_display_semantic_map_markers(publisher, my_markers):
	# create markerArray
	markerArray = MarkerArray()

	# add deleteall marker to the MarkerArray
	deleteall_marker = create_deleteall_marker()
	markerArray.markers.append(deleteall_marker)

	# loop over the markers and add them to the markerArray
	for this_marker in my_markers:
		# add line marker to the MarkerArray
		this_marker['scale'] = [0.2, 0.2, 1.0]
		this_marker['color'] = [1.0, 1.0, 0.0]
		line_marker = create_cylinder_marker(this_marker)
		markerArray.markers.append(line_marker)

		# add text marker to the MarkerArray
		text_marker = create_text_marker(this_marker)
		markerArray.markers.append(text_marker)

	# Publish the MarkerArray
	publisher.publish(markerArray)

#############################################
# setup publisher
#############################################
def setup():
	# set up the publisher and node
	topic = 'visualization_marker_array'
	publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)
	return publisher

#############################################
# if calling from the command line
#############################################
if __name__ == '__main__':
	rospy.init_node('rviz_marker_publisher')

	# setup node
	publisher = setup()

	# call display_markers function
	i = 1
	while (1):
		print("calling create_and_display_semantic_map_markers")
		
		# create a dummy marker list
		my_markers = []
		my_markers.append({'id':1, 'x':i,  'y':i,  'name':'point 1'})
		my_markers.append({'id':2, 'x':-3, 'y':-2, 'name':'point 2'})
		if i==3:
			i=0
		i += 1
		
		create_and_display_semantic_map_markers(publisher, my_markers)
		rospy.sleep(1)

