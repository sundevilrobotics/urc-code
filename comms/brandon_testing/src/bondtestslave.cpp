#include "ros/ros.h"
#include "std_msgs/Time.h"

#include <sstream>

#include <bondcpp/bond.h>
#include <ros/spinner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ProcessB", true);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {

        bond::Bond bond("example_bond_topic", "myBondId123456");
        bond.setHeartbeatTimeout(2.0);
        bond.start();
        // ... do things ...

        // fprintf(stdout, "Bond started\n");
        // for (size_t i = 0; i < 2000000000; i++)
        // {
        // }
        // fprintf(stdout, "Breaking bond\n");
        // bond.breakBond();
        // fprintf(stdout, "Bond stopped\n");
        // bond.waitUntilBroken(ros::Duration(-1.0));

        printf("B waiting for bond to be formed\n");
        if (!bond.waitUntilFormed(ros::Duration(5.0)))
        {
            ROS_ERROR("ERROR! Trying to bond again...");
            bond.breakBond();
        }
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::Time>("slavetime", 1000, true);
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
        printf("A has broken the bond\n");
    }
}