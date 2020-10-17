#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#define RAD2DEG(x)  ((x)*180./M_PI)

ros::Publisher prox_pub;
double prox_limit = 0.6;


double get_min_range(sensor_msgs::LaserScan  laser_scan_msg)
{
  int min_index = 0;

  for(int i=0; i < laser_scan_msg.ranges.size(); i++){
    if (!std::isnan(laser_scan_msg.ranges[i])){
      if ((laser_scan_msg.ranges[i] >= laser_scan_msg.range_min) && (laser_scan_msg.ranges[i] <= laser_scan_msg.range_max)){
        min_index = i;
        break;	
      }        
    }
  }

  for(int i= min_index + 1; i < laser_scan_msg.ranges.size(); i++){
    if (!std::isnan(laser_scan_msg.ranges[i])){
      if ((laser_scan_msg.ranges[i] >= laser_scan_msg.range_min) && (laser_scan_msg.ranges[i] <= laser_scan_msg.range_max)){
        if (laser_scan_msg.ranges[min_index] > laser_scan_msg.ranges[i]){
          min_index = i;
	}
      }
    }
  }

  return laser_scan_msg.ranges[min_index];

}


void scanCallback(const sensor_msgs::LaserScan scan_msg)
{

	//ROS_INFO("Angle range: %f, %f", RAD2DEG(scan_msg->angle_min), RAD2DEG(scan_msg->angle_max));
	std_msgs::Bool prox_msg;
        prox_msg.data = 0;

        if (get_min_range(scan_msg) <= prox_limit){
	  prox_msg.data = 1;
	}
         
       	prox_pub.publish(prox_msg);
        // std::cout << "Minimum range: " << get_min_range(scan_msg) << "\n";     

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_subscribe");
	ros::NodeHandle n;
        
	prox_pub = n.advertise<std_msgs::Bool>("/proximity_alert", 10);

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

	ros::spin();

	return 0;

}

