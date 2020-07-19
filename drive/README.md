# drive

# Dependencies:
- roboteq_driver (http://wiki.ros.org/roboteq_driver)
- *Note: roboteq_driver requires serial (https://github.com/wjwwood/serial.git)*

Author: Brandon Rice

Description: This package contains the code necessary to drive
the chassis of the 2019-2020 Sun Devil Robotics Club Mock Mars
Rover.


Use: To use, make sure both RoboteQ motor controllers and a
primary joystick are plugged into the microcontroller/laptop
running ROS. The tested configuration is one motor controller
and the joystick in the USB hub, plugged into one USB port.
The other motor controller is plugged into a remaining USB
port.

Then, simply run the teleop.launch file. This will set up two
Serial objects, one to /dev/ttyACM0 (the first recognized
motor controller) and one to /dev/ttyACM1 (the second motor
controller). The core method that is ran is Chassis::update(),
which manages a subscriber node on the topic /j0 (the joystick
input information) and sends the left and right axes data to
the correct RoboteQ serial commands (!G) for a tank/differential
drive scheme.

See chassis.h for more information.


Debugging: If the motor controllers and axes are inverted (left
stick controls the right motors), try reversing the USB ports
that the motor controllers are plugged into.
