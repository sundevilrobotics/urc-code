#include "ros/ros.h"
#include "std_msgs/Time.h"

#include <sstream>

#include <bondcpp/bond.h>
#include <ros/spinner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ProcessA", true);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {

        bond::Bond bond("example_bond_topic", "myBondId123456");
        bond.setHeartbeatTimeout(2.0);

        printf("A starting bond\n");
        bond.start();
        printf("A waiting for bond to be formed\n");
        if (!bond.waitUntilFormed(ros::Duration(5.0)))
        {
            ROS_ERROR("ERROR! Trying to bond again...");
            bond.breakBond();
        }
        printf("A waiting for bond to be broken\n");
        // // ... do things with B ...
        // bond.waitUntilBroken(ros::Duration(-1.0));

        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::Time>("mastertime", 1000, true);
        ros::Rate loop_rate(10);
        int count = 0;
        while (!bond.isBroken())
        {
            std_msgs::Time msg;

            msg.data = ros::Time::now();

            ROS_INFO("%f", msg.data.toSec());

            chatter_pub.publish(msg);

            ros::spinOnce();

            loop_rate.sleep();
            ++count;
        }
        printf("B has broken the bond\n");
    }
}