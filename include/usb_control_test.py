#!/usr/bin/env python
# -*- coding: utf-8 -*-

# モジュールのインポート
import math
import os
import sys
import time
import rospy
# import KMControllers
from roskeigan_motor_usb.msg import rot_state
from roskeigan_motor_usb.msg import motor_command
import usb_mode_test

# KeiganMotorのROSに対応させるクラス
class Keigan_Ros_Control():
    def __init__(self):
        # ノード名
        rospy.init_node('Keigan_Ros_Control')
        # Subscriberを宣言
        self.sub = rospy.Subscriber('motor_command', motor_command, self.callback)
        # Publisherを宣言
        self.pub = rospy.Publisher('rot_state', rot_state, queue_size = 10)
        # 変数の初期化
        self.state_pub = rot_state()

    # KeiganMotorの位置、速度、トルクを取得
    def setstate(self):
            self.state_pub.position = rospy.get_param('/mt_position/position', 0)
            self.state_pub.velocity = rospy.get_param('/mt_position/velocity', 0)
            self.state_pub.torque = rospy.get_param('/mt_position/torque', 0)
            self.pub.publish(self.state_pub)

    def callback(self, data):
        pass

if __name__ == '__main__':
    keigan_ros_control = Keigan_Ros_Control()
    # 制御周期
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    # [ctrl]+[c]でプログラムの終了するまでループ
    while not rospy.is_shutdown():
        keigan_ros_control.setstate()
        R.sleep()