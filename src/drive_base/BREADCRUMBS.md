# Breadcrumbs
This file is intended to serve as a casual usage/extension/maintenance guide for this unfinished chunk of software.

## To-Dos
- Somebody needs to take over as the maintainer of this package.  Fill out your info in `package.xml` in place of Jane Doe.
- Add the commands to `roboteq_diff_drive.cpp` to actually set the motor controllers' velocity, and any setup information, using the roboteq command server.  This may depend a bit on your hardware setup, so it has not yet been done.
- Calibrate the robot in the Configuration.h file.  This will depend a lot on what the robot ends up looking like.
- Add a publisher to publish any desired status info, also using the roboteq command server.
- Add debugging information.  This isn't super important, so don't prioritize it.
- Remove this file.  It doesn't belong inside a ROS package.  Copy it to wherever you need to.

## Codemap
`include/Configuration.h` : Robot Tuning Constants
`include/Constants.h` : Universal Robot Contents
`include/ErrorCodes.h` : Roboteq Error Codes
`include/Macros.h` : Useful macros
`include/RoboteqDevice.h` : Roboteq C++ driver
`srv/RoboteqCommand.srv` : Roboteq Command Server 
`src/RoboteqDevice.cpp` : Roboteq C++ driver
`src/roboteq_command_server.cpp` : ROS Roboteq Server
`src/roboteq_diff_drive.cpp` : ROS node to interact between cmd_vel and Roboteq Motor Controllers

## Usage
`roboteq_command_server` takes calls in the standard roboteq command format.
`roboteq_diff_drive` subscribes to /cmd_vel and runs the motor drivers.

