#!/usr/bin/env /python
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import time

class LedBuzzCtrl:
    def __init__(self):
        self.led_sub = rospy.Subscriber("/gpio/led", Bool, callback = self.led_callback)
        self.buzz_sub = rospy.Subscriber("/gpio/buzz", Bool, callback = self.buzz_callback)
        self.pin = [35, 38]

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.OUT, initial = GPIO.LOW)

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

def main():
    rospy.init_node("gpio_ctrl", anonymous = True)
    GPIO_Ctrl = LedBuzzCtrl()

if __name__ == "__main__":
    main()
