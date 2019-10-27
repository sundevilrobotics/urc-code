#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan

def in_range(range):
	for i in msg.ranges:
		if msg.ranges[i] < range:
			print 'Object in range'

def callback(msg):
	print 'Left distance reading: '
	print msg.ranges[0]
	print 'Straight ahead distance reading: '
	print msg.ranges[len(msg.ranges)/2]
	print 'Right distance reading: '
	print msg.ranges[len(msg.ranges)-1]

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()