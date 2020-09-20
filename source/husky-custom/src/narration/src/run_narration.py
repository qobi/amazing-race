#!/usr/bin/env python

#########################################################
# import libraries
#########################################################
import rospy
from std_msgs.msg import String

#########################################################
# execute command given by human
#########################################################
def print_narration(data):
	print data.data
	
#########################################################
# if calling from the command line...
#########################################################
if __name__ == '__main__':
	try:
		# initialize this node
		rospy.init_node('narration')

		# setup subscriber
		rospy.Subscriber('/narration', String, print_narration)

		# spin forever
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

