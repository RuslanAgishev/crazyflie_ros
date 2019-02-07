#!/usr/bin/env python

import numpy as np
import time
import sys

""" ROS """
import rospy
from geometry_msgs.msg import TransformStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
import crazyflie

def test(cf):
	cf.takeoff(targetHeight = TakeoffHeight, duration = TakeoffTime)
	time.sleep(TakeoffTime+1)
	cf.land(targetHeight = 0.0, duration = 10)
	time.sleep(10.0)

def led_control(cf):
	#cf.setParam("tf/state", 1)
	#time.sleep(1)
	#cf.setParam("tf/state", 2)
	#time.sleep(1)
	#cf.setParam("tf/state", 3)
	#time.sleep(1)
	cf.setParam("tf/state", 4)
	#time.sleep(1)
	#cf.setParam("tf/state", 0)
	time.sleep(1)


""" init """
TakeoffHeight = 1.8
TakeoffTime   = 6.0
toFly         = 1

try:
	cf_name = sys.argv[1]
except:
	cf_name = 'cf1'

print(cf_name)

cf1 = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
cf1.setParam("commander/enHighLevel", 1)
cf1.setParam("stabilizer/estimator", 2) # Use EKF
cf1.setParam("stabilizer/controller", 2) # Use Mellinger controller
led_control(cf1)

if toFly:
	print(cf_name+" takeoff")
	test(cf1)


