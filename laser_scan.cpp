#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


void scan_callback(sensor_msgs::LaserScan &msg){
	std::cout << "Left distance reading: ";
	std::cout << msg.ranges[0] << "\n";

	std::cout << "Straight ahead distance reading: ";
	std::cout << msg.ranges[msg.ranges.size()/2];

	std::cout << "Right distance reading: ";
	std::cout << msg.ranges[msg.ranges.size()-1];
}

int main(int argc, char **argv){

	//initialize the ROS node
	ros::init(argc, argv, "laser_scan_cpp");
	ros::NodeHandle n;

	//subscribe to the laser scanner topic
	scanSubscriber = n.subscribe("/scan", 10, scan_callback);

	ros::spin();
}