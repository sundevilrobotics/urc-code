#include <ros/ros.h>
#include "chassis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chassis teleop");

  ROS_INFO_STREAM("Teleop Mode Enabled");

  Chassis::tank_drive(1);

  ROS_INFO_STREAM("Teleop Mode Disabled");
}
