#include "chassis.h"

bool Chassis::inverted_controls = 0;
float Chassis::joyDataLeft = 0;
float Chassis::joyDataRight = 0;
float Chassis::set_linear_velocity = 0;
float Chassis::set_rotational_velocity = 0;
ros::Publisher Chassis::vel_pub;

// ros::NodeHandle Chassis::nh_l("","");
//
// Chassis::Chassis(int mc_1, int mc_2) : drive_left_control("drive_left_control", &str_msg),
//                                        drive_right_control("drive_right_control", &str_msg)
// {
//   cont_port_1 = "/dev/ttyACM" + std::string(mc_1);
//   cont_port_1 = "/dev/ttyACM" + std::string(mc_2);
//   nh_l.initNode(cont_port_1);
//   nh_r.initNode(cont_port_2);
// }

