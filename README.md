# Sun Devil Robotics Club's University Rover Challenge Code

Current Build Status:

(`develop (ROS Melodic)`) &nbsp; ![develop branch build status](https://github.com/sundevilrobotics/urc-code/actions/workflows/buildros.yml/badge.svg?branch=develop)

(`develop (ROS Noetic)`) &nbsp; ![develop branch build status](https://github.com/sundevilrobotics/urc-code/actions/workflows/buildrosnoetic.yml/badge.svg?branch=develop)

___

This folder contains the following ROS packages:
- ***[sdrc_urc](sdrc_urc)\****
- [sdrc_urc_arm](sdrc_urc_arm)
- [sdrc_urc_biology](sdrc_urc_biology)
- [sdrc_urc_bringup](sdrc_urc_bringup)
- [sdrc_urc_chassis](sdrc_urc_chassis)
- [sdrc_urc_comms](sdrc_urc_comms)
- [sdrc_urc_cv](sdrc_urc_cv)
- [sdrc_urc_description](sdrc_urc_description)
- [sdrc_urc_gazebo](sdrc_urc_gazebo)
- [sdrc_urc_gps](sdrc_urc_gps)
- [sdrc_urc_gui](sdrc_urc_gui)
- [sdrc_urc_jetson_nano](sdrc_urc_jetson_nano)
- [sdrc_urc_joystick](sdrc_urc_joystick)
- [sdrc_urc_lidar](sdrc_urc_lidar)
- [sdrc_urc_mapping](sdrc_urc_mapping)
- [sdrc_urc_navigation](sdrc_urc_navigation)
- [sdrc_urc_pi](sdrc_urc_pi)
- [sdrc_urc_zed2](sdrc_urc_zed2)

\* (The SDRC URC code ROS metapackage)
___

## Package: sdrc_urc_arm
Software to control the rover's arm.

## Package: sdrc_urc_biology
Software to control the rover's biology subsystems.
*Note: currently empty, needs updating.*

## Package: sdrc_urc_bringup
Software that launches the main subsystems. When in doubt, use these launch files! *Note: currently empty, needs updating.*

## Package: sdrc_urc_chassis
Software that interfaces with our chassis motor controllers. 
*Note: currently empty, needs updating.*

## Package: sdrc_urc_comms
Software that interfaces with our access points (routers) to allow us to detect
if the rover loses communications. This package also provides programs to drive
the access point motors to maintain line-of-sight with the base station. *Note: currently empty, needs updating.*

## Package: sdrc_urc_cv
Computer vision software, primarily used to detect the AR tags used in the
University Rover Challenge. *Note: currently empty, needs updating.*

## Package: sdrc_urc_description
The URDF definitions of the rover and ROS Gazebo launch files to spawn the rover
model in an empty world. *Note: currently empty, needs updating.*

## Package: sdrc_urc_gazebo
Software that launches the Gazebo Simulator with a variety of options of models
and terrains to simulate. *Note: currently empty, needs updating.*

## Package: sdrc_urc_gps
Software that interfaces with our GPS unit. *Note: currently empty, needs updating.*

## Package: sdrc_urc_gui
Software that defines and launches our graphical user interface (GUI). *Note: currently empty, needs updating.*

## Package: sdrc_urc_jetson_nano
Software that interfaces with the rover's Nvidia Jetson Nano microcontroller. *Note: currently empty, needs updating.*

## Package: sdrc_urc_joystick
Software that interfaces with a variety of joysticks and writes the values of
each joystick to
ROS topics. *Note: currently empty, needs updating.*

## Package: sdrc_urc_lidar
Software that interfaces with our LiDAR unit. *Note: currently empty, needs updating.*

## Package: sdrc_urc_mapping
Software used to provide simultaneous localization and mapping (SLAM) on the rover. *Note: currently empty, needs updating.*

## Package: sdrc_urc_navigation
Software used to autonomously navigate using the ROS Navigation Stack (NavStack). *Note: currently empty, needs updating.*

## Package: sdrc_urc_pi
Software that interfaces with the rover's Raspberry Pi microcontroller. *Note: currently empty, needs updating.*

## Package: sdrc_urc_zed2
Software that interfaces with the rover's ZED2 camera. *Note: currently empty, needs updating.*

___

In addition, this folder also contains the following:

- **install.sh** (A script that installs all dependencies needed for the
  sdrc_urc ROS package.)
- **uninstall.sh** (A script that uninstalls all dependencies needed for the
  sdrc_urc ROS package.)
- **extras/etc/udev/...** (Linux udev rules for remapping joysick controllers to new names.)


___

Are you an Arizona State University student interested in helping out with this project? Please visit https://sdrc.engineering.asu.edu/ for more information on how to join the Sun Devil Robotics Club!
