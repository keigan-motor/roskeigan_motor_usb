#!/usr/bin/env python
# -*- coding: utf-8 -*-

# モジュールのインポート
import math
import os
import sys
import time
from geometry_msgs.msg import Twist
import rospy
import KMControllers

# KeiganMotorのdevice_nameを指定
device_name = rospy.get_param('device_name')
motor = KMControllers.USBController(device_name)

# KeiganMotorのROSに対応させるクラス
class Keigan_Ros_Mode():
    def __init__(self):
        # ノード名
        rospy.init_node('Keigan_Ros_Mode')
        # [ctrl]+[c]でプログラムの終了　()内はプログラム終了後に実行される関数
        rospy.on_shutdown(self.shutdown)
        # Publisherの宣言
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.vel = rospy.Publisher('/vel', Twist, queue_size=5)
        # 実際に送信する「Twist」の「メッセージ」を「Twist」クラスから作成
        self.move_cmd = Twist()
        self.vel_com = Twist()
        # launchファイルで設定した速度を取得
        self.LINEAR_SPEED = rospy.get_param('linear_speed', 0)
        # X方向の速度を定義
        self.move_cmd.linear.x = self.LINEAR_SPEED

    # KeiganMotorの位置、速度、トルクを取得
    def on_motor_measurement_cb(self, measurement):
        if (measurement):
            print('mt_position {} '.format(measurement))
            # 取得した値をパラメータとして設定する
            rospy.set_param('/mt_position', measurement)
            self.ve = rospy.get_param('/mt_position/velocity', 0)
            # 取得した速度の値をX方向の速度に定義
            self.vel_com.linear.x = self.ve
            self.vel.publish(self.vel_com)

    # KeiganMotorをUSBモードに設定
    def setup_motor_usb_mode(self):
        if (motor):
            motor.on_motor_measurement_cb = None
            print('Change usb mode >>>>>>>>')
            type = motor.interface_type['USB']+motor.interface_type['HDDBTN']
            motor.interface(type)
            # motor.saveAllRegisters()
            time.sleep(1)
            time.sleep(5)
            motor.on_motor_measurement_cb = self.on_motor_measurement_cb
            print('>>>>>>>>Comp Change usb mode')
        else:
            sys.stderr.write('Not motor attached\n')

    # KeiganMotorを回転させる関数
    def play_sequence(self):
        motor.run(self.LINEAR_SPEED)
        self.cmd_vel.publish(self.move_cmd)

    # KeiganMotorとUSB接続
    def connection_usb_motor(self):
        print("connection_usb_motor")
        motor.on_motor_measurement_cb = self.on_motor_measurement_cb
        motor.enable()
        motor.presetPosition(0)
    
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
    keigan_ros_mode = Keigan_Ros_Mode()
    # 制御周期
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    keigan_ros_mode.connection_usb_motor()
    keigan_ros_mode.setup_motor_usb_mode()
    # [ctrl]+[c]でプログラムの終了するまでループ
    while not rospy.is_shutdown():
        keigan_ros_mode.play_sequence()
        keigan_ros_mode.on_motor_measurement_cb
        R.sleep()