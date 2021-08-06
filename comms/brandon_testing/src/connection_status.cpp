#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Duration.h"

ros::Duration delta;

void chatterCallback(const std_msgs::Duration::ConstPtr &msg)
{
    delta = msg->data;
    ROS_INFO("I heard: [%f]", msg->data.toSec());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "connectionnode");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("slavetimedelta", 1000, chatterCallback);

    ros::NodeHandle m;
    ros::Publisher chatter_pub = m.advertise<std_msgs::Bool>("slaveconnection", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::Bool msg;

        if (delta.toSec() > 2.0)
            msg.data = 0;
        else
            msg.data = 1;

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}