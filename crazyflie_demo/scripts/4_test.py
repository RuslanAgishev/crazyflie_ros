#!/usr/bin/env python

import numpy as np
import time
import sys
from threading import Thread

""" ROS """
import rospy
from geometry_msgs.msg import TransformStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
import crazyflie

def test(cf_name):
	cf1 = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
	cf1.setParam("commander/enHighLevel", 1)
	cf1.setParam("stabilizer/estimator", 2) # Use EKF
	cf1.setParam("stabilizer/controller", 2) # Use Mellinger controller
	time.sleep(0.1)
	print(cf_name+" takeoff")
	for t in range(3): cf1.takeoff(targetHeight = TakeoffHeight, duration = TakeoffTime)
	time.sleep(TakeoffTime)

	for t in range(3): cf1.land(targetHeight = 0.0, duration = TakeoffTime+3)
	time.sleep(3.0)

""" init """
TakeoffHeight = 1.0
TakeoffTime   = 6.0
toFly         = 1



if toFly:
    t1 = Thread(target=test, args=('cf1',))
    t2 = Thread(target=test, args=('cf2',))
    t3 = Thread(target=test, args=('cf3',))
    t4 = Thread(target=test, args=('cf4',))
    t1.start()
    t2.start()
    t3.start()
    t4.start()

