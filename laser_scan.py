#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan

def callback(msg):
	print 'Lidar reading directly to the right: '
	print msg.ranges[0]
	print 'Lidar reading directly ahead: '
	print msg.ranges[44]
	print 'Lidar reading directly to the left: '
	print msg.ranges[89]

rospy.init_node('scan_values')
sub = rospy.Subscriber('/base_scan', LaserScan, callback)
rospy.spin()