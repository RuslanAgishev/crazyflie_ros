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
toFly            = 1
vel_ctrl         = 0
vel_koef         = 4.0
pos_ctrl		 = 1
pos_coef         = 2.0
const_height	 = 1
human_imp        = False
theta_imp        = False
force_imp        = False
put_limits       = 0
TAKEOFFHEIGHT    = 1.45 # meters
HUMAN_Z_TO_LAND  = 0.6 # meters
TakeoffTime      = 5     # seconds
l                = 0.40     # distance between drones, meters
R_obstacles      = 0.25
limits           = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_negative  = np.array([ -1.7, -1.5, -0.1 ])
tacile_glove_on  = False
rotation 		 = 0
start_data_recording = False
killed_recorder = False
subject_name = "Test"



cf_names         = np.array(['cf1',
							 'cf2',
							 'cf3'])
human_name       = 'palm'
obstacle_names   = np.array([
							 'obstacle0',
							 'obstacle1',
							 'obstacle2',
							 'obstacle3',
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
		# cf1 = crazyflie.Crazyflie(cf_names[0], '/vicon/'+cf_names[0]+'/'+cf_names[0])
		# cf1.setParam("commander/enHighLevel", 1)
		# cf1.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		cf2 = crazyflie.Crazyflie(cf_names[1], '/vicon/'+cf_names[1]+'/'+cf_names[1])
		cf2.setParam("commander/enHighLevel", 1)
		cf2.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		# cf3 = crazyflie.Crazyflie(cf_names[2], '/vicon/'+cf_names[2]+'/'+cf_names[2])
		# cf3.setParam("commander/enHighLevel", 1)
		# cf3.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		#time to takeoff and select position for human
		time.sleep(TakeoffTime)

	# Objects init
	obstacle = np.array([])
	for i in range(len(obstacle_names)):
		obstacle = np.append(obstacle, swarmlib.Obstacle( obstacle_names[i], i, R_obstacles))
	drone1 = swarmlib.Drone(cf_names[0], obstacle, leader = True)
	drone2 = swarmlib.Drone(cf_names[1], obstacle)
	drone3 = swarmlib.Drone(cf_names[2], obstacle)
	human = swarmlib.Mocap_object(human_name)

	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		for i in range(len(obstacle)):
			obstacle[i].publish_position()

		if human_imp:
			# HUMAN IMPEDANCE
			hum_vel = swarmlib.hum_vel(human_pose)
			imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
			imp_pose_prev = imp_pose
			imp_vel_prev = imp_vel

		# if force_imp:
		# 	# FORCE IMPEDANCE
		# 	R0 = np.array([drone1.position()-obstacle[0].position()])[0][:2]
		# 	imp_pose, imp_vel, imp_time_prev = swarmlib.force_impedance_model(R0, imp_pose_prev, imp_vel_prev, imp_time_prev)
		# 	imp_pose_prev = imp_pose
		# 	imp_vel_prev = imp_vel

		# all drones follow a human with or wo impedance
		if vel_ctrl:
			if not initialized:
				# print "initialized"
				point_to_follow_pose_prev = np.array([drone1.position()[0],drone1.position()[1],TAKEOFFHEIGHT])
				human_pose_init = human.position() # np.array([human_pose[0],human_pose[1],human_pose[2]])
				time_prev = time.time()
				initialized = True
			cmd_vel = -vel_koef*(human_pose_init-human.position())
			np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.020), 0)
			time_now = time.time()
			point_to_follow_pose = point_to_follow_pose_prev + cmd_vel*(time.time()-time_prev)
			time_prev = time_now
			point_to_follow_pose_prev = point_to_follow_pose
			drone1.sp = point_to_follow_pose

		elif pos_ctrl:
			if not initialized:
				human_pose_init = human.position()
				if toFly:
					drone1_pose_init = drone1.position()
				else:
					drone1_pose_init = human.position() - np.array([1,0,0])
				initialized = True
			dx, dy = (human.position() - human_pose_init)[:2]
			drone1.sp = np.array([  drone1_pose_init[0] + pos_coef*dx,
									drone1_pose_init[1] + pos_coef*dy,
									TAKEOFFHEIGHT])
		else:
			drone1.sp = np.array([  human.position()[0] - 2*l,
									human.position()[1],
									human.position()[2] + 0.1])

		if const_height:
			drone1.sp[2] = TAKEOFFHEIGHT

		if put_limits:
			np.putmask(drone1.sp, drone1.sp >= limits, limits)
			np.putmask(drone1.sp, drone1.sp <= limits_negative, limits_negative)

		# drones forming equilateral triangle
		drone2.sp = drone1.sp + np.array([-0.86*l , l/2., 0])
		drone3.sp = drone1.sp + np.array([-0.86*l ,-l/2., 0])

		# ROTATION due to hand position
		if rotation:
			centroid = swarmlib.centroid_calc(drone1, drone2, drone3)
			drone1.sp = swarmlib.rotate(centroid, drone1, human)
			drone2.sp = swarmlib.rotate(centroid, drone2, human)
			drone3.sp = swarmlib.rotate(centroid, drone3, human)

		# OBSTACLEs
		# centroid_before_obstacles = swarmlib.centroid_calc(drone1, drone2, drone3)

		# if theta_imp:
		# 	for i in range(len(obstacle)):
		# 		drone1.sp, updated_1 = swarmlib.pose_update_obstacle_imp(drone1, obstacle[i], R_obstacles, delta_imp=False)
		# 		drone2.sp, updated_2 = swarmlib.pose_update_obstacle_imp(drone2, obstacle[i], R_obstacles, delta_imp=False)
		# 		drone3.sp, updated_3 = swarmlib.pose_update_obstacle_imp(drone3, obstacle[i], R_obstacles, delta_imp=False)
		# else:
		# 	for i in range(len(obstacle)):
		# 		drone1.sp, updated_1 = swarmlib.pose_update_obstacle(drone1, obstacle[i], R_obstacles)
		# 		drone2.sp, updated_2 = swarmlib.pose_update_obstacle(drone2, obstacle[i], R_obstacles)
		# 		drone3.sp, updated_3 = swarmlib.pose_update_obstacle(drone3, obstacle[i], R_obstacles)


		# # 2-nd and 3-rd drones avoid the 1-st
		# drone3.sp, updated_3 = swarmlib.pose_update_drone(drone3, drone1, 0.5*l)
		# drone2.sp, updated_2 = swarmlib.pose_update_drone(drone2, drone1, 0.5*l)

		# centroid_after_obstacles = swarmlib.centroid_calc(drone1, drone2, drone3)


		# TO FLY
		if toFly:
			drone1.fly()
			drone2.fly()
			drone3.fly()

		# TO VISUALIZE
		human.publish_position()
		drone1.publish_sp()
		drone2.publish_sp()
		drone3.publish_sp()
		path_limit = 1000 # [frames]
		drone1.publish_path(limit=path_limit)	#set -1 for unlimited path
		drone2.publish_path(limit=path_limit)
		drone3.publish_path(limit=path_limit)
		# swarmlib.publish_pose(centroid_after_obstacles, np.array([0,0,0,1]), '/centroid_after_obstacles')
		# swarmlib.publish_pose(centroid_before_obstacles, np.array([0,0,0,1]), '/centroid_before_obstacles')


		# if centroid_before_obstacles[1] - centroid_after_obstacles[1] > 0:
		# 	print 'move LEFT'
		# if centroid_before_obstacles[1] - centroid_after_obstacles[1] < 0:
		# 	print 'move RIGHT'
		# if centroid_before_obstacles[0] - centroid_after_obstacles[0] > 0:
		# 	print 'move BACK'
		# if centroid_before_obstacles[0] - centroid_after_obstacles[0] < 0:
		# 	print 'move FORWARD'

		if tacile_glove_on:
			prev_pattern_time = swarmlib.tactile_patterns(drone1.sp, drone2.sp, drone3.sp, prev_pattern_time)

		# Landing
		if toFly and human.position()[2]<HUMAN_Z_TO_LAND:
			print 'Landing!!!'
			drone1_landing_pose = drone1.position()
			drone2_landing_pose = drone2.position()
			drone3_landing_pose = drone3.position()
			while not rospy.is_shutdown():
				drone1.sp = drone1_landing_pose
				drone2.sp = drone2_landing_pose
				drone3.sp = drone3_landing_pose
				drone1_landing_pose[2] = drone1_landing_pose[2]-0.007
				drone2_landing_pose[2] = drone2_landing_pose[2]-0.007
				drone3_landing_pose[2] = drone3_landing_pose[2]-0.007
				drone1.fly()
				drone2.fly()
				drone3.fly()
				if drone1.sp[2]<-1.0 and drone2.sp[2]<-1.0 and drone2.sp[2]<-1.0:
					sleep(1)
					# cf1.stop()
					cf2.stop()
					# cf3.stop()
					print 'reached the floor, shutdown'
					rospy.signal_shutdown('landed')
				rate.sleep()


		rate.sleep()




		# drone1.sp = drone1.sp + imp_pose
		# drone2.sp = drone2.sp + imp_pose
		# drone3.sp = drone3.sp + imp_pose
		# drone2.sp[1] = drone2.sp[1] - imp_pose[0]*0.15
		# drone3.sp[1] = drone3.sp[1] + imp_pose[0]*0.15

		# # all drones follow a human
		# drone1.sp = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
		# drone2.sp = np.array([ drone1.sp[0] - l, drone1.sp[1] + l, drone1.sp[2] ])
		# drone3.sp = np.array([ drone1.sp[0] - l, drone1.sp[1] - l, drone1.sp[2] ])

		# # first is followed by the second
		# # Second is followed by the third
		# drone1.sp = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
		# drone2.sp = np.array([ drone1_pose[0] - l, drone1_pose[1] + l, drone1_pose[2] ])
		# drone3.sp = np.array([ drone2_pose[0]    , drone2_pose[1] -2*l, drone2_pose[2] ])