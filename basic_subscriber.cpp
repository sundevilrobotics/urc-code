#include <ros/ros.h>
#include <std_msgs/String.h>

void callback_receive_radio_data(const std_msgs::String& msg){
	ROS_INFO("Message from topic channel_1: %s", msg.data.c_str());
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "basic_subscriber");		// the node name does not have to be the same as the file name
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/channel_1", 10, callback_receive_radio_data);	
	// channel_1 is the topic the node is subscribing to
	// 10 is the queue size
	// the 3rd argument is the callback function. The node subscribes to the topic channel_1, receives the data, and
	//puts that data into the callback function

	ros::spin();		// spin keeps the node running until you shut it down
}