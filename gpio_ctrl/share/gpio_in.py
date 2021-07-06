#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from rospy.core import rospyinfo
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO

class GpioIn:
    def __init__(self):
        self.pin = [16, 18]
        self.none = None

        self.value1 = self.none
        self.value1_bat = self.none
        self.value1_mode = 0

        self.value2 = self.none
        self.value2_bat = self.none
        self.value2_mode = 0

        self.mode = 0
        self.start = 0

        self.mode_pub = rospy.Publisher('/mode', Int32, queue_size = 1)
        self.start_pub = rospy.Publisher('/start', Int32, queue_size = 1)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.IN)
        rospy.Timer(rospy.Duration(0.05), self.time_callback)
        rospy.spin()

    def time_callback(self, event):
        self.value1_bat = self.value1
        self.value1 = GPIO.input(self.pin[0])
        if self.value1 == GPIO.HIGH and self.value1_bat == GPIO.LOW:
            self.value1_mode = 1
        if self.value1_mode == 1 and self.value1 == GPIO.LOW and self.value1_bat == GPIO.HIGH:
            self.value1_mode = 0
            if(self.mode < 6):
                self.mode = self.mode + 1
            else:
                self.mode = 0
            self.mode_pub.publish(self.mode)
            # print("Button1 have been pressed")
        
        self.value2_bat = self.value2
        self.value2 = GPIO.input(self.pin[1])
        if self.value2 == GPIO.HIGH and self.value2_bat == GPIO.LOW:
            self.value2_mode = 1
        if self.value2_mode == 1 and self.value2 == GPIO.LOW and self.value2_bat == GPIO.HIGH:
            self.value2_mode = 0
            # if(self.start < 1):
            #     self.start = self.start + 1
            # else:
            #     self.start = 0
            self.start = 1
            self.start_pub.publish(self.start)
            # print("Button2 have been pressed")

def main():
    rospy.init_node("gpio_in", anonymous = True)
    GpioIn()

main()
