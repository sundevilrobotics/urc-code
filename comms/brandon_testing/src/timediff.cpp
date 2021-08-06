#include "ros/ros.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"

ros::Time lasttime;

void chatterCallback(const std_msgs::Time::ConstPtr &msg)
{
    lasttime = msg->data;
    ROS_INFO("I heard: [%f]", msg->data.toSec());
}

int main(int argc, char **argv)
{
    ros::Time::init();
    lasttime = ros::Time::now();

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("slavetime", 1000, chatterCallback);

    ros::NodeHandle m;
    ros::Publisher chatter_pub = m.advertise<std_msgs::Duration>("slavetimedelta", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::Duration delta = ros::Time::now() - lasttime;

        std_msgs::Duration msg;
        msg.data = delta;

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}