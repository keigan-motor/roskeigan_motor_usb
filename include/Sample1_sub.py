#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import sys
import time
from geometry_msgs.msg import Twist, Point, Quaternion
import rospy
import KMControllers

class Keigan_Ros_Mode_Sub():

    def __init__(self):
        rospy.init_node('Keigan_Ros_Mode_Sub')
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Subscriber('/cmd_vel', Twist, queue_size=5)

    def sub(self):
        position = rospy.get_param('/mt_position', 0)
        print position
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

if __name__ == '__main__':
    keigan_ros_mode_sub = Keigan_Ros_Mode_Sub()
    ROS_RATE = 30
    R = rospy.Rate(ROS_RATE)
    while not rospy.is_shutdown():
        keigan_ros_mode_sub.sub()
        R.sleep()
