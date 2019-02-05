#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import time

def joy_cb(data):
    global button
    button = data.buttons[1]

def buttons(button):
    rate = rospy.Rate(10)
    joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)

button = 0

if __name__ == '__main__':
    rospy.init_node('test')
    buttons(button)
    button_prev = button
    print button_prev - button
    rospy.spin()