#ifndef CHASSIS_H
#define CHASSIS_H

#include <ros/ros.h>
#include "Configuration.h"
#include <std_msgs/String.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>
#include "roboteq_msgs/Command.h"
#include <std_msgs/String.h>
#include <stdio.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"

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

    serial::Serial my_serial_l("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(10));
    serial::Serial my_serial_r("/dev/ttyACM1", 115200, serial::Timeout::simpleTimeout(10));

    std::cout << "Is the serial port open?";
    if(my_serial_l.isOpen())
      std::cout << " Yes." << std::endl;
    else
      std::cout << " No." << std::endl;

    std::string data;
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
      std::string result = my_serial_l.read(500);
       result += my_serial_r.read(500);

std::cout << "Bytes read: ";
std::cout << result.length() << ", String read: " << result << std::endl;

data = "!G 1 " + std::to_string(joyDataLeft * 1000) + "\r\n!G 2 " + std::to_string(joyDataLeft * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
  size_t bytes_wrote = my_serial_l.write(data);
data = "!G 1 " + std::to_string(joyDataRight * 1000) + "\r\n!G 2 " + std::to_string(joyDataRight * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
  bytes_wrote += my_serial_r.write(data);
        std::cout << "BYTES: " << bytes_wrote << " ; command: " << data << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
  }
};

#endif
