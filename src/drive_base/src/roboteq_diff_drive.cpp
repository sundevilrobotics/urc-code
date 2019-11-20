#include "ros/ros.h"
#include "drive_base/RoboteqCommand.h"



int main(int argc, char **argv)
{
    //Initialize ROS Node
    ros::init(argc, argv, "roboteq_diff_drive");
    ros::NodeHandle handle;
    ros::Subscriber subscriber = handle.subscribe()
    
}