#!/usr/bin/env python
# -*- coding: utf-8 -*-

# モジュールのインポート
import math
import os
import sys
import time
import rospy
#rospy.init_node('keigan_ros_node', log_level=rospy.DEBUG)

from roskeigan_motor_usb.msg import rot_state
from roskeigan_motor_usb.msg import motor_command
import utils

#keigan_ros_nodeに接続してkeigan_ros_nodeからモーター情報を取得したり、keigan_ros_nodeに制御コマンドを送るテスト
class keigan_ros_node_control_test():
    def __init__(self):
        # ノード名
        rospy.init_node('keigan_ros_node_control_test')
        # 回転情報を受信するSubscriberを宣言
        self.rotStateSub = rospy.Subscriber('rot_state', rot_state, self.on_rot_state)
        # 各モーターコマンドをトピックに発行するPublisherを宣言
        self.motorCommandPub = rospy.Publisher('/motor_command', motor_command, queue_size=10)

    # rot_stateトピックからモーターの位置情報取得時のイベント
    def on_rot_state(self, data):
        #KeiganMotorの位置、速度、トルク
        position = data.position
        velocity = data.velocity
        torque = data.torque
        rospy.logdebug('rotState',data)

    #モーターのコマンドを送信するラッパー
    def send_motor_command(self,command,*args):
        cmd_msg = motor_command()
        cmd_msg.command = command
        cmd_msg.args = args
        self.motorCommandPub.publish(cmd_msg)

    #-----------------------#
    #   モーターの制御サンプル
    #   info::コマンドと引数の説明は右記ローレベルAPIに準ずる　https://document.keigan-motor.com/software_dev/lowapis/motor_action
    #   info::テスト版の為、現時点で実装されている制御コードは、KMControllerROS.py::72行目以下を参照
    # -----------------------#
    def control_motor(self):
        # ---------無限回転---------#
        self.send_motor_command("enable")#モーターを動作許可する
        time.sleep(3)

        self.send_motor_command("run",utils.rpmToRadianSec(5))#正回転  5rpm
        time.sleep(10)

        self.send_motor_command("runReverse")#逆回転
        time.sleep(10)

        self.send_motor_command("speed", utils.rpmToRadianSec(20))  # 移動速度のみを20rpmに設定　info::指定しないと以前の速度が引き継がれる
        self.send_motor_command("runForward")#速度を維持したまま 正回転
        time.sleep(10)

        # ---------座標制御---------#
        self.send_motor_command("stop")# 停止（励磁あり）
        self.send_motor_command("presetPosition", 0)#座標を0にリセット
        self.send_motor_command("speed",utils.rpmToRadianSec(40))
        time.sleep(1)

        #絶対座標180度へ #info::sleep(5)以内に移動しきらないと、次のコマンドが発行され、目的座標に到着する前に移動されてしまう
        self.send_motor_command("moveTo",utils.degreeToRadian(180))
        time.sleep(5)

        # そこから相対で座標-90移動
        self.send_motor_command("moveBy",utils.degreeToRadian(90))
        time.sleep(5)

        # そこから相対で座標720度(2回転)移動
        self.send_motor_command("moveBy", utils.degreeToRadian(720))
        time.sleep(5)

        # 停止（励磁あり）
        time.sleep(5)
        self.send_motor_command("stop")

        # 停止（励磁無し）
        time.sleep(5)
        self.send_motor_command("disable") #モーターを動作不許可にする＞＞ 励磁も停止します


if __name__ == '__main__':
    node_instance = keigan_ros_node_control_test()
    # 制御周期
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    node_instance.control_motor()

    # [ctrl]+[c]でプログラムの終了するまでループ
    while not rospy.is_shutdown():
        R.sleep()

