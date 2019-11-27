#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

//float distance_at_index(sensor_msgs::PointCloud2 &msg, int x){
	//return msg.ranges[x];
	//return msg.data[x];
//}

void callback_scan(sensor_msgs::PointCloud2 msg){
	std::cout << msg.data[0] << std::endl;
	//std::cout << "Distance at minimum angle: " << distance_at_index(msg, 0) << "\n";
	//std::cout << "Distance at middle angle: " << distance_at_index(msg, msg.ranges.size()/2) << "\n";
	//std::cout << "Distance at maximum angle: " << distance_at_index(msg, msg.ranges.size()-1) << "\n";
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "lidar_scan");		
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/LeddarTech_1/scan_cloud", 10, callback_scan);		// change topic

	ros::spin();		
}
