#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import *
import math
import time
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from time import sleep
import message_filters
import matplotlib.pyplot as plt
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
from crazyflie_driver.msg import FullState
import geometry_msgs
import tf_conversions
import crazyflie
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})
import swarmlib

# PARAMETERs #############
toFly            = 0
vel_ctrl         = 1
vel_koef         = 4.0
pos_ctrl		 = 1
pos_coef         = 2.0
const_height	 = 1
joystick_imp     = False
put_limits       = 0
TAKEOFFHEIGHT    = 1.45 # meters
joystick_Z_TO_LAND  = 0.6 # meters
TakeoffTime      = 5     # seconds
l                = 0.40     # distance between drones, meters
R_obstacles      = 0.25
limits           = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_negative  = np.array([ -1.7, -1.5, -0.1 ])
tacile_glove_on  = False
rotation 		 = 0

cf_names         = np.array(['cf1',
							 'cf4',
							 'cf5'
							 ])
joystick_name       = 'palm'
obstacle_names   = np.array([
							 # 'obstacle0',
							 # 'obstacle1',
							 # 'obstacle2',
							 # 'obstacle3',
							 # 'obstacle4',
							 # 'obstacle5',
							 # 'obstacle6',
							 # 'obstacle7',
							 # 'obstacle8',
							 # 'obstacle9',
							 # 'obstacle10',
							 # 'obstacle11',
							 # 'obstacle12',
							 # 'obstacle13',
							 # 'obstacle14',
							 # 'obstacle15',
							 # 'obstacle16',
							 # 'obstacle17',
							 # 'obstacle18',
							 # 'obstacle19',
							 # 'obstacle20',
							 # 'obstacle21',
							 # 'obstacle22',
							 # 'obstacle23',
							 # 'obstacle24',
							 # 'obstacle25'
							 ])


# Variables #########################
initialized = False
prev_pattern_time = time.time()
imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

updated_1 = 0
# SetUp
if tacile_glove_on:
	swarmlib.startXbee()

if __name__ == '__main__':
	rospy.init_node('follow_multiple', anonymous=True)
	# TODO: swarmlib.SWARM_MANAGER(drone_list)

	if toFly:
		print "takeoff"
		""" takeoff """
		cf_list = []
		for cf_name in cf_names:
		    print "adding.. ", cf_name
		    cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
		    cf.setParam("commander/enHighLevel", 1)
		    cf.setParam("stabilizer/estimator",  2) # Use EKF
		    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
		    cf_list.append(cf)
		for t in range(3):
		    for cf in cf_list:
		        print "takeoff.. ", cf.prefix
		        cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 5.0)
		time.sleep(5.0)

	# Objects init
	obstacles = []
	for name in obstacle_names:
		obstacles.append( swarmlib.Obstacle(name) )

	drones = []
	for name in cf_names:
		drones.append( swarmlib.Drone(cf_names[0]) )
	drone1 = drones[0]

	joystick = swarmlib.Mocap_object(joystick_name)

	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		for obstacle in obstacles:
			obstacle.publish_position()

		if joystick_imp:
			# joystick IMPEDANCE
			hum_vel = swarmlib.hum_vel(joystick_pose)
			imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_joystick(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
			imp_pose_prev = imp_pose
			imp_vel_prev = imp_vel


		# all drones follow a joystick with or wo impedance
		if vel_ctrl:
			if not initialized:
				# print "initialized"
				point_to_follow_pose_prev = np.array([drone1.position()[0],drone1.position()[1],TAKEOFFHEIGHT])
				joystick_pose_init = joystick.position() # np.array([joystick_pose[0],joystick_pose[1],joystick_pose[2]])
				time_prev = time.time()
				initialized = True
			cmd_vel = -vel_koef*(joystick_pose_init-joystick.position())
			np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.020), 0)
			time_now = time.time()
			point_to_follow_pose = point_to_follow_pose_prev + cmd_vel*(time.time()-time_prev)
			time_prev = time_now
			point_to_follow_pose_prev = point_to_follow_pose
			drone1.sp = point_to_follow_pose

		elif pos_ctrl:
			if not initialized:
				joystick_pose_init = joystick.position()
				if toFly:
					drone1_pose_init = drone1.position()
				else:
					drone1_pose_init = joystick.position() - np.array([1,0,0])
				initialized = True
			dx, dy = (joystick.position() - joystick_pose_init)[:2]
			drone1.sp = np.array([  drone1_pose_init[0] + pos_coef*dx,
									drone1_pose_init[1] + pos_coef*dy,
									TAKEOFFHEIGHT])
		else:
			drone1.sp = np.array([  joystick.position()[0] - 2*l,
									joystick.position()[1],
									joystick.position()[2] + 0.1])

		if const_height:
			drone1.sp[2] = TAKEOFFHEIGHT

		if put_limits:
			np.putmask(drone1.sp, drone1.sp >= limits, limits)
			np.putmask(drone1.sp, drone1.sp <= limits_negative, limits_negative)

		# drones forming equilateral triangle
		drones[1].sp = drone1.sp + np.array([-0.86*l , l/2., 0])
		drones[2].sp = drone1.sp + np.array([-0.86*l ,-l/2., 0])

		# ROTATION due to hand position
		if rotation:
			centroid = swarmlib.centroid_calc(drone1, drones[1], drones[2])
			drone1.sp = swarmlib.rotate(centroid, drone1, joystick)
			drones[1].sp = swarmlib.rotate(centroid, drones[1], joystick)
			drones[2].sp = swarmlib.rotate(centroid, drones[2], joystick)

		# TO FLY
		if toFly:
			for drone in drones: drone.fly()

		# TO VISUALIZE
		joystick.publish_position()
		for drone in drones: drone.publish_sp()

		path_limit = 1000 # [frames], set -1 for unlimited path
		for drone in drones: drone.publish_path_sp(limit=path_limit)


		if tacile_glove_on:
			prev_pattern_time = swarmlib.tactile_patterns(drone1.sp, drone2.sp, drone3.sp, prev_pattern_time)

		# Landing
		if toFly and joystick.position()[2]<joystick_Z_TO_LAND:
			print 'Landing!!!'
			while not rospy.is_shutdown():
				for drone in drones:
					drone.sp -= 0.007
					drone.fly()
				if drone1.sp[2]<-1.0:
					sleep(1)
					for cf in cf_list: cf.stop()
					print 'reached the floor, shutdown'
					rospy.signal_shutdown('landed')
				rate.sleep()


		rate.sleep()


