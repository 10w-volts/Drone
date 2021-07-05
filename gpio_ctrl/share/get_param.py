#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from rospy.core import rospyinfo
import time

if __name__ == '__main__':
    while True:
        param = rospy.get_param('/mode')
        print('param is' + str(param))
        time.sleep(0.05)