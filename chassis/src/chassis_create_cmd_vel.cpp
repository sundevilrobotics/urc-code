#include <ros/ros.h>
#include "chassis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chassis create cmd_vel");

  ROS_INFO_STREAM("Starting conversion of joystick input to chassis cmd_vel");

  // Subscriber node to handle joystick input data
  ros::NodeHandle joy_handle;
  ros::Subscriber sub = joy_handle.subscribe("joy0", 1000, Chassis::create_cmd_vel);

  while(ros::ok())
  {
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Ending conversion of joystick input to chassis cmd_vel");
}
