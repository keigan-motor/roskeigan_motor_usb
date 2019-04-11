#!/usr/bin/env python
# -*- coding: utf-8 -*-

# モジュールのインポート
import math
import os
import sys
import time
import rospy

#sys.path.append('/home/harada/catkin_ws/devel/lib/python2.7/dist-packages')#todo::ros実行時のpythonで含まれない
# import pprint
# pprint.pprint(sys.path)

from roskeigan_motor_usb.msg import motor_command
import KMControllerROS

# KeiganMotorのdevice_nameを指定
device_name = rospy.get_param('device_name')
motor = KMControllerROS.USBController(device_name)

# KeiganMotorの仮想ノード。モーターの数だけ存在する
# keigan_ros_nodeから操作する
class Keigan_Ros_Node():
    def __init__(self):
        # ノード初期化
        rospy.init_node('keigan_ros_node')
        # Publisherを宣言
       # self.pub = rospy.Publisher('motor_command', motor_command, queue_size = 10)
        self.pub = rospy.Publisher('/mt_position', motor_command, queue_size=10)
        # 変数の初期化
        self.command_pub = motor_command()
        # [ctrl]+[c]でプログラムの終了　()内はプログラム終了後に実行される関数
        rospy.on_shutdown(self.shutdown)

    def on_motor_measurement_cb(self, measurement):
        if (measurement):
            # 取得した値をパラメータとして設定する
            rospy.set_param('/mt_position', measurement)

    # KeiganMotorとUSB接続
    def connection_usb_motor(self):
        print("connection_usb_motor")
        motor.on_motor_measurement_cb = self.on_motor_measurement_cb
        motor.enable()
        motor.presetPosition(0)

    # motor_commandのデータをPublish
    def publish(self):
        # device_name
        self.command_pub.device_name = rospy.get_param('device_name')
        # speed (速度の大きさ)
        self.command_pub.speed = rospy.get_param('speed',0)
        # preset (位置をプリセット(原点))
        self.command_pub.preset = rospy.get_param('preset',0)
        # readPositionOffset (位置のプリセットに関するオフセット量)
        self.command_pub.readPositionOffset = rospy.get_param('readPositionOffset',0)
        # run (速度制御)
        self.command_pub.run = rospy.get_param('run',0)
        # moveTo (位置制御(絶対位置))
        self.command_pub.moveTo = rospy.get_param('moveTo',0)
        # moveBy (位置制御(相対位置))
        self.command_pub.moveBy = rospy.get_param('moveBy',0)
        # hold (トルク制御)
        self.command_pub.hold = rospy.get_param('hold',0)
        # 上記をPublish
        self.pub.publish(self.command_pub)

    # [ctrl]+[c]でプログラムの終了した際の関数
    def shutdown(self):
        motor.on_motor_measurement_cb = None
        # Motorを停止
        motor.stop()
        # Motorの励磁停止
        motor.free()
        # Motor動作不許可
        motor.disable()
        rospy.loginfo("Stopping the motor...")
    

if __name__ == '__main__':
    node_instance = Keigan_Ros_Node()
    # 制御周期
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    node_instance.connection_usb_motor()
    while not rospy.is_shutdown():
        node_instance.publish()
        R.sleep()