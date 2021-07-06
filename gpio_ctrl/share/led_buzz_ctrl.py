#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO
import time

class LedBuzzCtrl:
    def __init__(self):
        self.led_sub = rospy.Subscriber("/gpio/led", Bool, callback = self.led_callback)
        self.buzz_sub = rospy.Subscriber("/gpio/buzz", Bool, callback = self.buzz_callback)
        
        self.mode_sub = rospy.Subscriber("/mode", Int32, callback = self.mode_callback)
        self.start_sub = rospy.Subscriber("/start", Int32, callback = self.start_callback)
        
        self.pin = [29, 31]
        #self.pin = [7, 11]

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.OUT, initial = GPIO.LOW)

        self.startup()

        rospy.spin()
    
    def led_callback(self, data):
        if(data.data == True):
            GPIO.output(self.pin[0], GPIO.HIGH)
        else:
            GPIO.output(self.pin[0], GPIO.LOW)

    def buzz_callback(self, data):
        if(data.data == True):
            GPIO.output(self.pin[1], GPIO.HIGH)
        else:
            GPIO.output(self.pin[1], GPIO.LOW)

    def mode_callback(self, data):
        for i in range(data.data + 1):
            GPIO.output(self.pin[0], GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(self.pin[0], GPIO.LOW)
            time.sleep(0.1)

    def start_callback(self, data):
        for i in range(data.data + 1):
            GPIO.output(self.pin[0], GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(self.pin[0], GPIO.LOW)
            time.sleep(0.1)
    
    def startup(self):
        GPIO.output(self.pin[0], GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.pin[0], GPIO.LOW)
        time.sleep(0.1)

        GPIO.output(self.pin[0], GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.pin[0], GPIO.LOW)
        time.sleep(0.1)

        GPIO.output(self.pin[0], GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.pin[0], GPIO.LOW)
        time.sleep(0.1)

def main():
    rospy.init_node("gpio_ctrl", anonymous = True)
    GPIO_Ctrl = LedBuzzCtrl()

main()
