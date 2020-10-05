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
#include <math.h>

class Chassis
{

public:

  // floats to store most-recent data from joystick subscriber
  inline static float joyDataLeft;
  inline static float joyDataRight;

  // Joystick data subscriber callback to assign member variables with data
  static void chatterCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {

    joyDataLeft = joy->axes[1];  // Left Y Axis
    joyDataRight = joy->axes[4]; // Right Y Axis

    joyDataLeft = 2.0/3.1415 * atan(1.8 * pow(joyDataLeft, 1.8) );
    joyDataRight = 2.0/3.1415 * atan(1.8 * pow(joyDataRight, 1.8) );

  }

  static void tank_drive(bool inverted = 0)
  {
    //
    inverted_controls = inverted;

    // Subscriber node to handle joystick input data
    ros::NodeHandle joy_handle;
    ros::Subscriber sub = joy_handle.subscribe("/joy0", 1000, chatterCallback);

    // Serial objects used to communicate with RoboteQ motor controllers
    serial::Serial my_serial_l("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(10)); // 10 ms timeout
    serial::Serial my_serial_r("/dev/ttyACM1", 115200, serial::Timeout::simpleTimeout(10)); // 10 ms timeout

    // Initial debug statements to check for left motor controller port opened
    std::cout << "Is the serial port open?";
    if(my_serial_l.isOpen())
      std::cout << " Yes." << std::endl;
    else
      std::cout << " No." << std::endl;

    // String used to fill with data and send over serial
    std::string data;
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
      std::string result = my_serial_l.read(500); // Read "all" of left motor controller feedback buffer
      result += my_serial_r.read(500);  // Read "all" of right motor controller feedback buffer

      // Display feedback info from both motor controllers
      std::cout << "Bytes read: ";
      std::cout << result.length() << ", String read: " << result << std::endl;

      // Send new joystick data to motor controllers

      size_t bytes_wrote = 0;

      if(!inverted_controls) // Normal control layout
      {
        // Write to left motor controller
        data = "!G 1 " + std::to_string(-joyDataLeft * 1000) + "\r\n!G 2 " + std::to_string(-joyDataLeft * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
        bytes_wrote = my_serial_l.write(data);

        // Write to right motor controller
        data = "!G 1 " + std::to_string(joyDataRight * 1000) + "\r\n!G 2 " + std::to_string(joyDataRight * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
        bytes_wrote += my_serial_r.write(data);
      }
      else // Inverted (Drive the rover backward!)
      {
        // Write to left motor controller
        data = "!G 1 " + std::to_string(joyDataRight * 1000) + "\r\n!G 2 " + std::to_string(joyDataRight * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
        bytes_wrote = my_serial_l.write(data);

        // Write to right motor controller
        data = "!G 1 " + std::to_string(-joyDataLeft * 1000) + "\r\n!G 2 " + std::to_string(-joyDataLeft * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
        bytes_wrote += my_serial_r.write(data);
      }

      // Verify the data was sent correctly
      std::cout << "BYTES: " << bytes_wrote << " ; command: " << data << std::endl;

      // Delay for threaded looping
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  private:
    static bool inverted_controls;
};

#endif
