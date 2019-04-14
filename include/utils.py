#!/usr/bin/env python
# -*- coding: utf-8 -*-


#角度の単位変換　degree >> radian
def degreeToRadian(degree):
    return degree * 0.017453292519943295;

#角度の単位変換　radian >> degree
def radianToDegree(radian):
    return radian / 0.017453292519943295;

#速度 rpm >> radian/sec
def rpmToRadianSec(rpm):
    # 速度 rpm ->radian / sec(Math.PI * 2 / 60)
    return rpm * 0.10471975511965977;

#速度 radian/sec >> rpm
def radianSecToRpm(radianSec):
    # 速度 rpm ->radian / sec(Math.PI * 2 / 60)
    return radianSec / 0.10471975511965977;