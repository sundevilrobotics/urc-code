#ifndef CHASSIS_H
#define CHASSIS_H

#include <ros/ros.h>
#include "Configuration.h"
#include <std_msgs/String.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>

class Chassis
{
  // private:
  //   // ODOM, Current Orientation Data
  //   static std::string cont_port_1;
  //   static std::string cont_port_2;
  //
  //   static ros::NodeHandle nh_l;
  //   static ros::NodeHandle nh_r;
  //
  //   static ros::Publisher drive_left_control;
  //   static ros::Publisher drive_right_control;
  //
  //   static std::string left_command;
  //   static std::string right_command;
  //
  //   static std_msgs::String str_msg_l;
  //   static std_msgs::String str_msg_r;
  //
  // public:
  //   Chassis(int = 0, int = 1);
  //
  //   static void update()
  //   {
  //     ros::Publisher pub_l = nh_l.advertise<std_msgs::String>("drive_left_control", 1000);
  //     ros::Publisher pub_r = nh_r.advertise<std_msgs::String>("drive_right_control", 1000);
  //
  //     while (ros::ok())
  //     {
  //       pub_l.publish(str_msg_l);
  //       pub_r.publish(str_msg_r);
  //
  //       ros::spinOnce();
  //     }
  //   }
public:

  inline static float joyDataLeft;
  inline static float joyDataRight;

  static void chatterCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    joyDataLeft = joy->axes[1];
    joyDataRight = joy->axes[4];
  }

  static void update()
  {
    ros::NodeHandle joy_handle;
    ros::Subscriber sub = joy_handle.subscribe("/j0", 1000, chatterCallback);



    ros::NodeHandle n;
    ros::Publisher chatter_pub_l = n.advertise<std_msgs::String>("drive_left_control", 1000);
    ros::Publisher chatter_pub_r = n.advertise<std_msgs::String>("drive_right_control", 1000);
  ros::Rate loop_rate(1000);
  int count = 0;
while (ros::ok())
{
  /**
   * This is a message object. You stuff it with data, and then publish it.
   */
   std_msgs::String msg_l;
   std_msgs::String msg_r;

   std::stringstream ss_l;
   std::stringstream ss_r;
  ss_l << "!g 1 " << joyDataLeft * 200 << "_" << std::endl;
  // ss << "!g 2 " << joyData * 200 << "_" << std::endl;

  ss_r << "!g 1 " << joyDataRight * 200 << "_" << std::endl;

  msg_l.data = ss_l.str();

  msg_r.data = ss_r.str();


  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
   chatter_pub_l.publish(msg_l);
   chatter_pub_r.publish(msg_r);

  ros::spinOnce();

  loop_rate.sleep();
  ++count;
}

  }
};

#endif
