#include <ros/ros.h>
#include "chassis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive");

  ROS_INFO_STREAM("Teleop Mode Enabled");

  Chassis::update();

  ROS_INFO_STREAM("Teleop Mode Disabled");
}
