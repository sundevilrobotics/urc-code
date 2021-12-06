#!/usr/bin/env python3

from led.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Changing color to %s"%(req.color))
    return AddTwoIntsResponse("Changed color!")

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to change color.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
