#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


#define RAD2DEG(x)  ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{

	ROS_INFO("Scanning...");
	//ROS_INFO("Angle range: %f, %f", RAD2DEG(scan_msg->angle_min), RAD2DEG(scan_msg->angle_max));

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_subscribe");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

	ros::spin();

	return 0;

}

