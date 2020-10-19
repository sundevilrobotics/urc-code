#ifndef CHASSIS_H
#define CHASSIS_H

#include <ros/ros.h>
#include "Configuration.h"
#include <std_msgs/String.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>
//#include "roboteq_msgs/Command.h"
#include <std_msgs/String.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include <math.h>
#include <utility>      // std::pair, std::make_pair
#include <exception>
#include <ros/console.h>


class Chassis
{

public:


  // Joystick data subscriber callback to assign member variables with data
  static void chatterCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {

    joyDataLeft = joy->axes[1];  // Left Y Axis
    joyDataRight = joy->axes[4]; // Right Y Axis

    joyDataLeft = 2.0/3.1415 * atan(1.8 * pow(joyDataLeft, 1.8) );
    joyDataRight = 2.0/3.1415 * atan(1.8 * pow(joyDataRight, 1.8) );

  }

  static void create_cmd_vel(const sensor_msgs::Joy::ConstPtr& joy)
  {
    ros::NodeHandle nh;
    joyDataLeft = joy->axes[1];  // Left Y Axis
    joyDataRight = joy->axes[4]; // Right Y Axis

    vel_pub = nh.advertise<geometry_msgs::Twist>("chassis/cmd_vel", 10);

    std::string teleop_control_scheme;
    nh.getParam("teleop_control_scheme", teleop_control_scheme);

        geometry_msgs::Twist twist;


    if(teleop_control_scheme == "tank")
    {
      twist.linear.x = (joyDataRight + joyDataLeft) / 2; // Set forward/backward axis data
      twist.angular.z = (joyDataRight - joyDataLeft) / 2; // Set rotation axis data
    }
    else if(teleop_control_scheme == "arcade")
    {
      twist.linear.x = joy->axes[1]; // Set forward/backward axis data
      twist.angular.z = joy->axes[0]; // Set rotation axis data
    }
    else
    {
      ROS_ERROR("Error: Joystick control scheme not specified as parameter in launch file or via rosrun!");
    }
    
    vel_pub.publish(twist);
  }

  static void handle_cmd_vel(const geometry_msgs::Twist& cmd_vel)
  {
    /* NOTE: THIS TAKES A VELOCITY AND UPDATES THE CHASSIS! TO CHANGE THE CONTROLLER
     *       SPEED CURVE, ETC., UPDATE joystick_handler!
     */

    set_linear_velocity = 0.8 * MAX_SPEED * cmd_vel.linear.x;     // m/s per Configuration.h
    set_rotational_velocity = 0.8 * MAX_TURN * cmd_vel.angular.z; // rad/s per Configuration.h
  }

  static void tank_drive(bool inverted = 0)
  {
    // Update inverted_controls each time
    inverted_controls = inverted;

    // Subscriber node to handle joystick input data
    ros::NodeHandle joy_handle;
    ros::Subscriber sub = joy_handle.subscribe("/j0", 1000, chatterCallback);

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

  static std::pair<float, float> calculate_differential_vel(float linear_velocity, float rotational_velocity)
  {
    // Equations from: http://www.openrobots.org/morse/doc/1.3/user/actuators/v_omega_diff_drive.html

    std::pair<float, float> diff_vel;

    float l_vel = (linear_velocity - (1/2 * TRACK_WIDTH) * rotational_velocity) / WHEEL_DIAMETER;
    float r_vel = (linear_velocity + (1/2 * TRACK_WIDTH) * rotational_velocity) / WHEEL_DIAMETER;

    diff_vel.first = l_vel;
    diff_vel.second = r_vel;

    ROS_INFO("%f, %f", l_vel, r_vel);

    return diff_vel;
  }

  static void control(bool inverted = 0)
  {
    // Update inverted_controls each time
    inverted_controls = inverted;

    // Subscriber node to handle joystick input data
    ros::NodeHandle joy_handle;
    ros::Subscriber sub = joy_handle.subscribe("/chassis/cmd_vel", 1000, handle_cmd_vel);

    bool initialized_motor_controllers = 0;

    // Serial objects used to communicate with RoboteQ motor controllers
    serial::Serial my_serial_l, my_serial_r; 

    while(!initialized_motor_controllers)
    {
      try
      {
        my_serial_l.setPort("/dev/ttyACM0");
        my_serial_l.setBaudrate(115200);
        my_serial_r.setPort("/dev/ttyACM1");
        my_serial_r.setBaudrate(115200);

        // FIXME: Add setTimeout to 10 ms?

        my_serial_l.open();
        my_serial_r.open();
        if(my_serial_l.isOpen() && my_serial_r.isOpen())
          initialized_motor_controllers = 1;
        else
          ROS_ERROR("Unhandled Exception: Serial connections not initialized.");
      }
      catch(std::exception &e)
      {
        ROS_ERROR("Unhandled Exception: %s", e.what());
      }
    }

    // Initial debug statements to check for left motor controller port opened
    std::cout << "Is the serial port open?";
    if(my_serial_l.isOpen())
      std::cout << " Yes." << std::endl;
    else
      std::cout << " No." << std::endl;

    // String used to fill with data and send over serial
    std::string data;
    ros::Rate loop_rate(1000);

    std::pair<float, float> diff_vel;

    while(ros::ok())
    {

      diff_vel = calculate_differential_vel(set_linear_velocity, set_rotational_velocity);
      std::string result;
      try
      {
        result = my_serial_l.read(500); // Read "all" of left motor controller feedback buffer
        result += my_serial_r.read(500);  // Read "all" of right motor controller feedback buffer
      }
      catch(std::exception &e)
      {
        result = "";
        std::string exc = "Unhandled Exception: ";
        exc += e.what();
        ROS_ERROR("Unhandled Exception: %s", e.what());
      }


      // Display feedback info from both motor controllers
      std::cout << "Bytes read: ";
      std::cout << result.length() << ", String read: " << result << std::endl;

      // Send new joystick data to motor controllers

      size_t bytes_wrote = 0;

      try
      {
        if(!inverted_controls) // Normal control layout
        {
          // Write to left motor controller
          data = "!G 1 " + std::to_string(-diff_vel.first) + "\r\n!G 2 " + std::to_string(-diff_vel.first * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
          bytes_wrote = my_serial_l.write(data);

          // Write to right motor controller
          data = "!G 1 " + std::to_string(diff_vel.second * 1000) + "\r\n!G 2 " + std::to_string(diff_vel.second * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
          bytes_wrote += my_serial_r.write(data);
        }
        else // Inverted (Drive the rover backward!)
        {
          // Write to left motor controller
          data = "!G 1 " + std::to_string(diff_vel.second * 1000) + "\r\n!G 2 " + std::to_string(diff_vel.second * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
          bytes_wrote = my_serial_l.write(data);

          // Write to right motor controller
          data = "!G 1 " + std::to_string(-diff_vel.first * 1000) + "\r\n!G 2 " + std::to_string(-diff_vel.first * 1000) + "\r\n";// + std::to_string(joyDataLeft) + "\r\n";
          bytes_wrote += my_serial_r.write(data);
        }      
      }
      catch(std::exception &e)
      {
        std::string exc = "Unhandled Exception: ";
        exc += e.what();
        ROS_ERROR("Unhandled Exception: %s", e.what());
      }


      // Verify the data was sent correctly
      std::cout << "BYTES: " << bytes_wrote << " ; command: " << data << std::endl;

      // Delay for threaded looping
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  protected:
    static bool inverted_controls;
    // floats to store most-recent data from joystick subscriber
    static float joyDataLeft;
    static float joyDataRight;
    static float set_linear_velocity;
    static float set_rotational_velocity;
    static ros::Publisher vel_pub;
};

#endif
