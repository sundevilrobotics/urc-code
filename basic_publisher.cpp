#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_publisher");		// the node name does not have to be the same as the file name
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::String>("/channel_1", 10);	// /channel_1 is the topic name, 10 is the queue size 

	ros::Rate rate(1);		// publish once per second

	while (ros::ok()){
		std_msgs::String msg;
		msg.data = "This is the channel 1 news, with your host, Charles Wolfe";
		pub.publish(msg);
		rate.sleep();
	}
}