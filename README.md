# Sun Devil Robotics Club's University Rover Competition Code

This code contains the following packages:
- [drive](drive)
- [image_processing](image_processing)
- [learning_joy](learning_joy)
- [sdrc_arm_v1](sdrc_arm_v1)

___

## Package: drive
Software that interfaces with our RoboteQ FBL2360 to control the four motors in a tank configuration. Currently only supports open-loop command with no sensor feedback. See issues section for more details. *Note: will soon be updated to package: "Chassis."*

## Package: image_processing
AR tag tracking software (from the ROS package ar_track_alvar) and other OpenCV functionality for onboard video and image processing. *Note: will soon be updated to just a node in package: "JetsonNano."*

## Package: learning_joy
Software that interfaces with our joystick controllers and publishes their data to topics. *Note: will soon be updated to sub-package: "Control/Joystick."*

## Package: sdrc_arm_v1
Software to control our arm. *Note: will soon be updated to package: "Arm."*


___

Are you an Arizona State University student interested in helping out with this project? Please visit https://sdrc.engineering.asu.edu/ for more information on how to join the Sun Devil Robotics Club!
