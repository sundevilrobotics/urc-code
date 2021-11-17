#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist
import datetime

# To detect when the robot starts to move to shut down the node
# vel = 0

# def callback(data):
#     if abs(data.linear.x) > 0.1:
#         vel = data.linear.x

def set_nav_goal():
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.init_node('PubNavGoal', anonymous=True)

    # Get robot's velocity
    # rospy.Subscriber('/cmd_vel', Twist, callback)

    # Create the Nav goal object
    goal = MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id = 'map'
    goal.goal.target_pose.pose.position.x = float(input('Set goal x position (Default = 0.0): '))
    goal.goal.target_pose.pose.position.y = float(input('Set goal y position (Default = 0.3): '))
    goal.goal.target_pose.pose.position.z = 0.0
    goal.goal.target_pose.pose.orientation.w = float(input('Set goal orientation (w) (Default = 1.0): '))
    goal.goal_id.id = datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")

    # Keep publishing the goal until the robot notices and starts to drive
    rate = rospy.Rate(100)  # 10hz
    # while not rospy.is_shutdown() and vel == 0:
    #     pub.publish(goal)
    #     rate.sleep()

    # for i in range(100):
    #     pub.publish(goal)
    #     rate.sleep()

    pub.publish(goal)


if __name__ == '__main__':
    try:
        set_nav_goal()
    except rospy.ROSInterruptException:
        pass
