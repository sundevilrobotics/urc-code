#!/usr/bin/env python3

import rospy
from std_msgs.msg import twist

# teleop_cb takes /cmd_vel data and sets the motors to their correct speed (and
# angle once swerve drive is implemented)
def teleop_cb(data):
    a = 0

def teleop():
    rospy.init_node('sdrc_urc_chassis_teleop')
    # Create srv to allow for start, stop, and E-stop calls
    # Create sub for /cmd_vel that calls teleop_cb()
    rospy.Subscriber("/cmd_vel", twist, teleop_cb)
    # 
    rospy.spin()

if __name__ == "__main__":
    teleop()
