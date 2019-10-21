#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


void scan_callback(sensor_msgs::LaserScan &msg){
	std::cout << "Reading at 1 degree: ";
	std::cout << msg.ranges[0] << "\n";

	std::cout << "Reading at 160 degrees: ";
	std::cout << msg.ranges[362];

	std::cout << "Reading at 320 degrees: ";
	std::cout << msg.ranges[725];
}

int main(int argc, char **argv){

	//initialize the ROS node
	ros::init(argc, argv, "laser_scan_cpp");
	ros::NodeHandle n;

	//subscribe to the laser scanner topic
	scanSubscriber = n.subscribe("/scan", 10, scan_callback);

	ros::spin();
}