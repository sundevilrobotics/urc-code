#include <ros/ros.h>
#include "chassis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chassis control");

  ROS_INFO_STREAM("Chassis Control Enabled");

  Chassis::control(1);

  ROS_INFO_STREAM("Chassis Control Disabled");
}
