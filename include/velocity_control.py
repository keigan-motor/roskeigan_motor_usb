#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import sys
import time
from geometry_msgs.msg import Twist
import rospy
import KMControllers

device_name = rospy.get_param('device_name')
motor = KMControllers.USBController(device_name)

class Keigan_Ros_Mode():
    def __init__(self):
        rospy.init_node('Keigan_Ros_Mode')
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.vel = rospy.Publisher('/vel', Twist, queue_size=5)
        self.move_cmd = Twist()
        self.vel_com = Twist()
        self.LINEAR_SPEED = rospy.get_param('linear_speed', 0)
        self.move_cmd.linear.x = self.LINEAR_SPEED

    def on_motor_measurement_cb(self, measurement):
        if (measurement):
            print('mt_position {} '.format(measurement))
            rospy.set_param('/mt_position', measurement)
            self.ve = rospy.get_param('/mt_position/velocity', 0)
            self.vel_com.linear.x = self.ve
            self.vel.publish(self.vel_com)

    def setup_motor_usb_mode(self):
        if (motor):
            motor.on_motor_measurement_cb = None
            print('Change usb mode >>>>>>>>')
            type = motor.interface_type['USB']+motor.interface_type['HDDBTN']
            motor.interface(type)
            motor.saveAllRegisters()
            time.sleep(1)
            time.sleep(5)
            motor.on_motor_measurement_cb = self.on_motor_measurement_cb
            print('>>>>>>>>Comp Change usb mode')
        else:
            sys.stderr.write('Not motor attached\n')

    def play_sequence(self):
        motor.run(self.LINEAR_SPEED)
        self.cmd_vel.publish(self.move_cmd)

    def connection_usb_motor(self):
        print("connection_usb_motor")
        motor.on_motor_measurement_cb = self.on_motor_measurement_cb
        motor.enable()
        motor.presetPosition(0)
    
    def shutdown(self):
        motor.on_motor_measurement_cb = None
        motor.stop()
        motor.free()
        motor.enable()
        rospy.loginfo("Stopping the motor...")

if __name__ == '__main__':
    keigan_ros_mode = Keigan_Ros_Mode()
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    keigan_ros_mode.connection_usb_motor()
    keigan_ros_mode.setup_motor_usb_mode()
    while not rospy.is_shutdown():
        keigan_ros_mode.play_sequence()
        keigan_ros_mode.on_motor_measurement_cb
        R.sleep()