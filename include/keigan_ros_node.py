#!/usr/bin/env python
# -*- coding: utf-8 -*-

# モジュールのインポート
import math
import os
import sys
import time
import rospy

from roskeigan_motor_usb.msg import motor_command
from roskeigan_motor_usb.msg import rot_state
import KMControllerROS

# KeiganMotorの仮想ノード。モーターの数だけ存在する
class Keigan_Ros_Node():
    def __init__(self):
        # ノード初期化
        rospy.init_node('keigan_ros_node')
        # KeiganMotorのdevice_nameを指定
        self.motor = KMControllerROS.USBController(rospy.get_param('~device_name'))

        # Publisherを宣言
        self.rotStatePub = rospy.Publisher('/rot_state', rot_state, queue_size=10)
        # Subscriberを宣言
        self.motorCommandSub = rospy.Subscriber('motor_command', motor_command, self.motorCommandSubCallback)

        # 変数の初期化
        self.rotState = rot_state()
        self.rotStatePub.publish(self.rotState)
        self.motorCommand = motor_command()

        # [ctrl]+[c]でプログラムの終了　()内はプログラム終了後に実行される関数
        rospy.on_shutdown(self.shutdown)

    def on_motor_measurement_cb(self, measurement):
        if (measurement):
            self.rotState.position=measurement["position"]
            self.rotState.velocity = measurement["velocity"]
            self.rotState.torque = measurement["torque"]
            self.rotStatePub.publish(self.rotState)

            rospy.logdebug('rotState', self.rotState.position)

    #受信時
    def motorCommandSubCallback(self, data):
        pass

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
    node_instance = Keigan_Ros_Node()
    # 制御周期
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    node_instance.connection_usb_motor()
    while not rospy.is_shutdown():
        R.sleep()