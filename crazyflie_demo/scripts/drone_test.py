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

def test(cf_name):
	num_commands = 3
	cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
	cf.setParam("commander/enHighLevel", 1)
	cf.setParam("stabilizer/estimator", 2) # Use EKF
	cf.setParam("stabilizer/controller", 2) # Use Mellinger controller
	for t in range(num_commands): cf.takeoff(targetHeight = TakeoffHeight, duration = TakeoffTime)
	time.sleep(TakeoffTime)
	# for t in range(num_commands): cf.goTo(goal = [0.0, 0.0, -TakeoffHeight+0.05], yaw=0.0, duration = 5.0, relative = True)
	# time.sleep(7.0)
	# for t in range(num_commands): cf.goTo(goal = [0.0, 0.0, TakeoffHeight], yaw=0.0, duration = 3.0, relative = True)
	# time.sleep(3.0)
	# for t in range(num_commands): cf.goTo(goal = [-0.5, -0.5, 0.0], yaw=0.0, duration = 2.0, relative = True)
	# time.sleep(3.0)
	for t in range(num_commands): cf.land(targetHeight = 0.0, duration = TakeoffTime+3)


""" init """
TakeoffHeight = 0.6
TakeoffTime   = 5.0
toFly         = 1

try:
	cf_name = sys.argv[1]
except:
	cf_name = 'cf1'

if toFly:
	print(cf_name+" takeoff")
	test(cf_name)

