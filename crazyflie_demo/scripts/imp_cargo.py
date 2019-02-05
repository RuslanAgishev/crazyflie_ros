#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import *
import time
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from time import sleep
import message_filters
import matplotlib.pyplot as plt

from message_filters import TimeSynchronizer, Subscriber
import numpy as np

import swarmlib

from crazyflie_driver.msg import FullState
import geometry_msgs
import tf_conversions

import crazyflie
from sensor_msgs.msg import Joy

np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()
rotated = False
impedance = False
DIST = 0.5 # start distance between drones


def joy_cb(joystick):
	global raw
	global column
	global width
	global imp_on
	global imp_off
	raw = joystick.buttons[3]
	column = joystick.buttons[2]
	width = joystick.axes[1]
	imp_on = joystick.buttons[0]
	imp_off = joystick.buttons[1]


# def tag_game(human, cf1):
def tag_game(human, cf1, cf2):
	human_pose = swarmlib.get_coord(human)
	drone1_pose = swarmlib.get_coord(cf1)
	drone2_pose = swarmlib.get_coord(cf2)

	hum_vel = swarmlib.hum_vel(human_pose)
	l = 1.0
	# HUMAN IMPEDANCE
	global imp_pose_prev
	global imp_vel_prev
	global imp_time_prev
	global rotated
	global impedance
	global DIST

	# define joystick buttons state
	try:
		if raw==1:
			rotated = True
		elif column==1:
			rotated = False
		if width == 1:
			DIST += 0.01
		elif width == -1:
			DIST -= 0.01
		elif imp_on == 1:
			impedance = 1
		elif imp_off == 1:
			impedance = 0
	except:
		pass

	if impedance:
		print('Impedance control')
		imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
		imp_pose_prev = imp_pose
		imp_vel_prev = imp_vel
	else:
		print('No impedance control')
		imp_pose = np.array([0,0,0])

	if rotated:
		drone1_pose_goal = np.array([  human_pose[0] - l,
								   	   human_pose[1],
								       human_pose[2] + 0.1])
		drone2_pose_goal = np.array([  drone1_pose_goal[0] - DIST,
									   drone1_pose_goal[1],
									   human_pose[2] + 0.1])
		drone1_pose_goal[0] += abs( imp_pose[1]*0.5 )
		drone2_pose_goal[0] -= abs( imp_pose[1]*0.5 )

	else:
		drone1_pose_goal = np.array([  human_pose[0] - l,
									   human_pose[1] - DIST/2,
									   human_pose[2] + 0.1])
		drone2_pose_goal = np.array([  drone1_pose_goal[0],
									   drone1_pose_goal[1] + DIST,
									   human_pose[2] + 0.1])
		drone1_pose_goal[1] -= abs( imp_pose[0]*0.4 )
		drone2_pose_goal[1] += abs( imp_pose[0]*0.4 )

	# TO FLY
	swarmlib.publish_goal_pos(drone1_pose_goal, 0, "/crazyflie13")
	swarmlib.publish_goal_pos(drone2_pose_goal, 0, "/crazyflie15")

	# TO VISUALIZE
	swarmlib.publish_pose(drone1_pose_goal, 0, "drone1_pose_goal")
	swarmlib.publish_pose(drone2_pose_goal, 0, "drone2_pose_goal")
	swarmlib.publish_pose(human_pose, 0, "human_pose")


def follower():
	start_time = time.time()

	human_sub = message_filters.Subscriber('/vicon/human/human', TransformStamped)
	cf1_sub = message_filters.Subscriber('/vicon/crazyflie13/crazyflie13', TransformStamped)
	cf2_sub = message_filters.Subscriber('/vicon/crazyflie15/crazyflie15', TransformStamped)
	joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)
	
	ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub], 10, 5)

	ts.registerCallback(tag_game)
	rospy.spin()


if __name__ == '__main__':

	rospy.init_node('follow_multiple', anonymous=True)


	rospy.loginfo("Takeoff")
	HEIGHT = 0.3
	cf1 = crazyflie.Crazyflie("crazyflie13", "/vicon/crazyflie13/crazyflie13")
	cf1.setParam("commander/enHighLevel", 1)
	cf1.takeoff(targetHeight = HEIGHT, duration = 1.0)
	cf2 = crazyflie.Crazyflie("crazyflie15", "/vicon/crazyflie15/crazyflie15")
	cf2.setParam("commander/enHighLevel", 1)
	cf2.takeoff(targetHeight = HEIGHT, duration = 1.0)
	time.sleep(3.0)


	rospy.loginfo("Following human!")
	try:
		follower()
	except KeyboardInterrupt:
		pass


	rospy.loginfo("Try to land")

	try:
		cf1.stop()
	except:
		pass
	try:
		cf2.stop()
	except:
		pass

