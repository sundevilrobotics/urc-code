#!/usr/bin/python
import rospy
import serial
import struct
import numpy as np
import time
import math
from sensor_msgs.msg import Joy
vel_multi = 1

class arm:
    def __init__(self):
        self.velocities = {
            'wrist_roll': 0,
            'wrist_pitch': 0,
            'elbo_pitch': 0,
            'shou_pitch': 0,
            'shou_yaw': 0,
            'gripper': 0,
        }
     
        baudrate = rospy.get_param('~baudrate', 9600)
        self.serialDev = serial.Serial(baudrate=baudrate)
        self.serialDev.port = rospy.get_param("~serial_device")
        self.serialDev.open()
        self.arm_sub = rospy.Subscriber("/joy_arm", Joy, self.arm_joy_callback)

    def write_serial(self):
      # Execute arm position
        rospy.loginfo('velocities:%s\n' % self.velocities)
        encoded_vel = struct.pack("<ffffff",
                                        self.velocities['wrist_roll'],
                                        self.velocities['wrist_pitch'],
                                        self.velocities['elbo_pitch'],
                                        self.velocities['shou_pitch'],
                                        self.velocities['shou_yaw'],
                                        self.velocities['gripper'],
                                        )
        self.serialDev.write(encoded_vel)

    def arm_joy_callback(self, data):
        self.velocities['wrist_roll'] = 0
        self.velocities['wrist_pitch'] = 0
        self.velocities['elbo_pitch'] = 0
        self.velocities['shou_pitch'] = 0
        self.velocities['shou_yaw'] = 0
        self.velocities['gripper'] = 0
        global vel_multi

        #slow slew mode
        if data.buttons[5] and vel_multi == 1:
            vel_multi = 0.5

        elif data.buttons[5] and vel_multi == 0.5:
            vel_multi = 1

        rospy.loginfo('vel_multi: %s', vel_multi)

        # R thumbstick left/right
        self.velocities['shou_yaw'] = math.floor(data.axes[3] * -2000 * vel_multi)

        # R thumbstick up/down
        self.velocities['shou_pitch'] = math.floor(data.axes[4] * -600 * vel_multi)

        # L thumbstick up/down
        self.velocities['elbo_pitch'] = math.floor(data.axes[1] * -4000 * vel_multi)

        # button 6 = wrist down
        # button 4 = wrist up
        if data.buttons[5]:
            self.velocities['gripper'] = 1
        if data.buttons[4]:
            self.velocities['gripper'] = -1
        #if data.buttons[0]:
            #self.velocities['shou_pitch'] = -400
        #if data.buttons[1]:
            #self.velocities['shou_pitch'] = 400



            # MOVE ARM
        self.write_serial()

def main():
    rospy.init_node("sdrc_arm_v1")
    controller = arm()
    rospy.spin()
