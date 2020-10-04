# Package Folder: control

This folder contains the following ROS packages:
- [gps](gps)
- [image_processing](image_processing)
- [jetson_nano](jetson_nano)
- [joystick_handler](joystick_handler)
- [lidar](lidar)

___

## Package: gps
Software that interfaces with our GPS unit.

## Package: image_processing *currently depreciated*
AR tag tracking software (from the ROS package ar_track_alvar) and other OpenCV functionality for onboard video and image processing. *Note: will soon be updated to just a node in package: "jetson_nano"*

## Package: jetson_nano
Software to control and perform rover-based operations on our Jetson Nano.

## Package: joystick_handler
Software that interfaces with our joystick controllers and publishes their data to topics. *Note: make sure to add the file(s) in /extras/etc/udev/... if you want run our joysick_handler software.*

## Package: lidar
Software that interfaces with our LIDAR unit.