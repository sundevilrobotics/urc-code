#include "ros/ros.h"
#include "drive_base/RoboteqCommand.h"
#include "geometry_msgs/Twist.h"
#include "Configuration.h"

void callback(const geometry_msgs::Twist::ConstPtr& msg){
    double linear = msg->linear.x;
    double angular = msg->angular.z;
    double leftVelocity;  //Velocity 1
    double rightVelocity;  //Velocity 2
    leftVelocity = linear - TRACK_WIDTH * angular/2;
    rightVelocity = linear + TRACK_WIDTH * angular/2;
    
}


int main(int argc, char **argv)
{
    //Initialize ROS Node
    ros::init(argc, argv, "roboteq_diff_drive");
    ros::NodeHandle handle;
    ros::Subscriber subscriber = handle.subscribe("/cmd_vel", 1000, callback);
    
}