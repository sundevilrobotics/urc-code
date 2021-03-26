#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "Configuration.h"
// #include "joystick/RoboteqCommand.h"

// To use: run:
// roslaunch turtle_joy.launch

// Or alternatively:
// rosrun joy joynode with rosrun
// and
// rosrun joystick turtle_teleop_joy

enum joystickInputs
{
  // Digital
  A,
  B,
  X,
  Y,
  L1,
  R1,
  L2,
  R2,
  Share,
  Options,
  Home,
  L3,
  R3,
  // Analog
  LSX,
  LSY,
  L2Analog,
  RSX,
  RSY,
  R2Analog,
  HatX,
  HatY
};

// struct joystick_values
// {
//   float value[21];
// };


class JoystickHandler
{
public:
  JoystickHandler(int);
  float getValue(joystickInputs);
  int id;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  float values[21];

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

// Set-up subscribers/publishers in constructor
JoystickHandler::JoystickHandler(int joynum):
  linear_(1),
  angular_(1)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  // Get joystick info from /joy
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("j"+std::to_string(joynum), 10, &JoystickHandler::joyCallback, this);

  std::string topic_name;
  nh_.getParam("joy_pub_topic", topic_name);

  // Set joystick values in /cmd_vel
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_name, 1);

  id = joynum;
}

float JoystickHandler::getValue(joystickInputs i)
{
  return values[i];
}

void JoystickHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(id == 0)
  {
    std::cout << "\033[1;33m";
  }
  else if(1)
  {
    std::cout << "\033[1;36m";
  }
  for(int i = 0; i < 8; i++)
  {
    values[i] = joy->axes[i];
    printf("%f ", values[i]);
  }
  for(int i = 0; i < 13; i++)
  {
    values[i+8] = joy->buttons[i];
    printf("%i ", (int)values[i+8]);
  }
  std::cout << "\033[0m\n";
  // printf("\n");

  geometry_msgs::Twist twist;
  twist.angular.z = joy->axes[angular_]; // Set rotation axis data
  twist.linear.x = joy->axes[linear_]; // Set forward/backward axis data
  vel_pub_.publish(twist);
}

void callback(const geometry_msgs::Twist::ConstPtr& msg){
    double linear = msg->linear.x;
    double angular = msg->angular.z;
    double leftVelocity;  //Velocity 1
    double rightVelocity;  //Velocity 2
    leftVelocity = linear - TRACK_WIDTH * angular/2;
    rightVelocity = linear + TRACK_WIDTH * angular/2;

    // ROS_INFO("%f, %f", leftVelocity, rightVelocity);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_handler");

  JoystickHandler joy_handler(0);
  JoystickHandler joy_handler1(1);

  // Handler for setting calcualted velocity values
  // ros::NodeHandle vel_handle;
  // ros::Subscriber vel_sub = vel_handle.subscribe("/cmd_vel", 1000, callback);

  ros::spin();
}
