#!/usr/bin/env python
# -*- coding: utf-8 -*-

# モジュールのインポート
import math
import os
import sys
import time

#==============remote debug code added==================================================================:
# sys.path.append("pydevd-pycharm.egg")
# import pydevd_pycharm
#
# pydevd_pycharm.settrace('192.168.24.21', port=12345, stdoutToServer=True,
# stderrToServer=True)
import json
#================================================================================================

import rospy
from roskeigan_motor_usb.msg import rot_state
from roskeigan_motor_usb.msg import motor_command
import KMControllerROS
import utils

# KeiganMotorの仮想ノード。モーターの数だけ存在する
class keigan_ros_node():
    def __init__(self):
        # ノード初期化
        rospy.init_node('keigan_ros_node')
        # KeiganMotorのdevice_nameを指定
        self.motor = KMControllerROS.USBController(rospy.get_param('~device_name'))
        # Publisherを宣言
        self.rotStatePub = rospy.Publisher('/rot_state', rot_state, queue_size=10)
        # Subscriberを宣言
        rospy.Subscriber('motor_command', motor_command, self.on_motor_command)

        # 変数の初期化
        self.rotState = rot_state()
        self.rotStatePub.publish(self.rotState)

        # [ctrl]+[c]でプログラムの終了　()内はプログラム終了後に実行される関数
        rospy.on_shutdown(self.shutdown)

    def on_motor_measurement_cb(self, measurement):
        if (measurement):
            self.rotState.position=measurement["position"]
            self.rotState.velocity = measurement["velocity"]
            self.rotState.torque = measurement["torque"]
            self.rotStatePub.publish(self.rotState)


    def on_motor_command(self, motor_command):
        if (motor_command):
            rospy.loginfo("command %s %s", motor_command.command, motor_command.args)
            if(hasattr(self.motor, motor_command.command)):
                motor_method=None
                try:
                    motor_method = getattr(self.motor, motor_command.command)
                except AttributeError:
                    rospy.loginfo("Err command not found %s", motor_command.command)
                    return
                motor_method(*motor_command.args)

    # KeiganMotorとUSB接続
    def connection_usb_motor(self):
        print("connection_usb_motor")
        self.motor.on_motor_measurement_cb = self.on_motor_measurement_cb
        self.motor.enable()
        self.motor.presetPosition(0)

    # [ctrl]+[c]でプログラムの終了した際の関数
    def shutdown(self):
        self.motor.on_motor_measurement_cb = None
        # Motorを停止
        self.motor.stop()
        # Motorの励磁停止
        self.motor.free()
        # Motor動作不許可
        self.motor.disable()
        rospy.loginfo("Stopping the motor...")

if __name__ == '__main__':
    node_instance = keigan_ros_node()
    # 制御周期
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    node_instance.connection_usb_motor()
    while not rospy.is_shutdown():
        R.sleep()