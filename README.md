# Sun Devil Robotics Club's University Rover Competition Code

This folder contains the following ROS packages:
- [arm](arm)
- [biology](biology)
- [chassis](chassis)
- [comms](comms)
- [control](control)*
- [sim](sim)

___

## Package: arm
Software to control our arm. *Note: currently is supported with rospy.*

## Package: biology
Software that interfaces with our biology systems' microcontroller. *Note: currently empty, needs updating.*

## Package: chassis
Software that interfaces with our RoboteQ FBL2360 to control the four motors in a tank configuration. Currently only supports open-loop command with no sensor feedback. See issues section for more details.

## Package: comms
AR tag tracking software (from the ROS package ar_track_alvar) and other OpenCV functionality for onboard video and image processing. *Note: will soon be updated to just a node in package: "jetson_nano".*

## Package Folder: control
A folder with that contains packages which handle all control elements (i.e. sensors, input devices, and the actual microcontroller code).

## Package: sim
Software used to simulate our packages. *Note: currently empty, needs updating.*

___

In addition, this folder also contains the following:

- **install.sh** (A script that installs all dependencies needed for urc-code.)
- **uninstall.sh** (A script that uninstalls all dependencies needed for urc-code.)
- **extras/etc/udev/...** (Linux udev rules for remapping joysick controllers to new names.)

___

Are you an Arizona State University student interested in helping out with this project? Please visit https://sdrc.engineering.asu.edu/ for more information on how to join the Sun Devil Robotics Club!
