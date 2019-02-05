#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from math import *
import time
from std_srvs.srv import Empty
from std_msgs.msg import Float32
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
from threading import Thread


imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

imp_delta_pose1 = np.array( [0,0,0] ); imp_delta_vel1 = np.array( [0,0,0] ); imp_delta_time1 = time.time()
imp_delta_pose2 = np.array( [0,0,0] ); imp_delta_vel2 = np.array( [0,0,0] ); imp_delta_time2 = time.time()
imp_delta_pose3 = np.array( [0,0,0] ); imp_delta_vel3 = np.array( [0,0,0] ); imp_delta_time3 = time.time()

imp_pose_from_theta = None
imp_theta_prev = 0
imp_omega_prev = 0
imp_theta_time_prev = time.time()

imp_pose_from_theta2 = None
imp_theta_prev2 = 0
imp_omega_prev2 = 0
imp_theta_time_prev2 = time.time()

imp_pose_from_theta3 = None
imp_theta_prev3 = 0
imp_omega_prev3 = 0
imp_theta_time_prev3 = time.time()

flew_in1 = 0
flew_in2 = 0
flew_in3 = 0

cf1_name = 'cf1'
cf2_name = 'cf2'
cf3_name = 'cf3'
human_name = 'palm'



toFly = 0
human_imp = 0
delta_imp = 1
theta_imp = 0

path1 = Path()
path2 = Path()
path3 = Path()



# def tag_game(human, cf1, obstacle1):
# def tag_game(human, cf1, cf2):
#def tag_game(human, cf1, cf2, cf3, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9):
def tag_game(human, cf1, cf2, cf3, obstacle1, obstacle2):
	human_pose = swarmlib.get_coord(human)
	obstacle1_pose = swarmlib.get_coord(obstacle1)
	obstacle2_pose = swarmlib.get_coord(obstacle2)
	drone1_pose = swarmlib.get_coord(cf1)
	drone2_pose = swarmlib.get_coord(cf2)
	drone3_pose = swarmlib.get_coord(cf3)

	Obstacles_map = np.array([obstacle1_pose[:2],
							 #obstacle2_pose[:2],
							 # obstacle3_pose,
							 # obstacle4_pose,
							 # obstacle5_pose,
							 # obstacle6_pose,
							 # obstacle7_pose,
							 # obstacle8_pose,
							 # obstacle9_pose
							])

	l = 0.35
	R_obstacles= 0.25

	hum_vel = swarmlib.hum_vel(human_pose)

	drone1_w, drone1_vel = swarmlib.drone_w(drone1_pose, drone1_pose - obstacle1_pose)
	drone2_w, drone2_vel = swarmlib.drone_w(drone2_pose, drone2_pose - obstacle1_pose)
	drone3_w, drone3_vel = swarmlib.drone_w(drone3_pose, drone3_pose - obstacle1_pose)

	# HUMAN IMPEDANCE
	global imp_pose_prev
	global imp_vel_prev
	global imp_time_prev
	imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
	imp_pose_prev = imp_pose
	imp_vel_prev = imp_vel


	# all drones follow a human with impedance
	drone1_pose_goal = np.array([  human_pose[0] - 1.0*l       ,
								   human_pose[1]               ,
								   human_pose[2] + 0.1         ])
	drone2_pose_goal = np.array([ drone1_pose_goal[0] - l      ,
								  drone1_pose_goal[1] + l      ,
								  drone1_pose_goal[2]          ])
	drone3_pose_goal = np.array([ drone1_pose_goal[0] - l      ,
								  drone1_pose_goal[1] - l      ,
								  drone1_pose_goal[2]          ])
	if human_imp:
		drone1_pose_goal = drone1_pose_goal + imp_pose
		drone2_pose_goal = drone2_pose_goal + imp_pose
		drone3_pose_goal = drone3_pose_goal + imp_pose
		drone2_pose_goal[1] = drone2_pose_goal[1] - imp_pose[0]*0.15
		drone3_pose_goal[1] = drone3_pose_goal[1] + imp_pose[0]*0.15
	# drone1_pose_goal = np.array([  human_pose[0]               ,
	# 							   human_pose[1] - 2.0*l       ,
	# 							   human_pose[2] + 0.1         ])
	# drone2_pose_goal = np.array([ drone1_pose_goal[0] - l      ,
	# 							  drone1_pose_goal[1] - l      ,
	# 							  drone1_pose_goal[2]          ])
	# drone3_pose_goal = np.array([ drone1_pose_goal[0] + l      ,
	# 							  drone1_pose_goal[1] - l      ,
	# 							  drone1_pose_goal[2]          ])
	

	drone1_pose_goal, updated_1, delta1, theta1_sp = swarmlib.pose_update_obstacle(Obstacles_map, drone1_pose_goal, R_obstacles)
	drone2_pose_goal, updated_2, delta2, theta2_sp = swarmlib.pose_update_obstacle(Obstacles_map, drone2_pose_goal, R_obstacles)
	drone3_pose_goal, updated_3, delta3, theta3_sp = swarmlib.pose_update_obstacle(Obstacles_map, drone3_pose_goal, R_obstacles)

	# OBSTACLE DELTA IMPEDANCE
	global imp_delta_pose1; global imp_delta_vel1; global imp_delta_time1
	global imp_delta_pose2; global imp_delta_vel2; global imp_delta_time2
	global imp_delta_pose3; global imp_delta_vel3; global imp_delta_time3
	imp_delta_pose1, imp_delta_vel1, imp_delta_time1 = swarmlib.impedance_obstacle_delta(delta1, imp_delta_pose1, imp_delta_vel1, imp_delta_time1)
	imp_delta_pose2, imp_delta_vel2, imp_delta_time2 = swarmlib.impedance_obstacle_delta(delta2, imp_delta_pose2, imp_delta_vel2, imp_delta_time2)
	imp_delta_pose3, imp_delta_vel3, imp_delta_time3 = swarmlib.impedance_obstacle_delta(delta3, imp_delta_pose3, imp_delta_vel3, imp_delta_time3)
	if delta_imp:
		drone1_pose_goal += imp_delta_pose1
		drone2_pose_goal += imp_delta_pose2
		drone3_pose_goal += imp_delta_pose3

	# # OBSTACLE THETA IMPEDANCE
	drone1_pose_goal_prev = drone1_pose_goal
	global flew_in1
	global imp_pose_from_theta
	global imp_theta_prev
	global imp_omega_prev
	global imp_theta_time_prev
	global current_obstacle_index1
	flew_in1, closest_obstacle_index1 = swarmlib.obstacle_status(Obstacles_map, drone1_pose_goal_prev, imp_pose_from_theta, human_pose, R_obstacles, flew_in1)
	if flew_in1==1: current_obstacle_index1 = closest_obstacle_index1
	# in the circle-like vicinity of the obstacle
	if flew_in1 > 0:
		if flew_in1 == 1: # crossed the circle near the obstacle
			#print "Drone 1 is near the obstacle "+str(current_obstacle_index1+1)
			imp_theta = theta1_sp
			imp_theta_prev = imp_theta
			imp_omega_prev = drone1_w[2]
		else:
			imp_theta, imp_omega, imp_theta_time_prev = swarmlib.impedance_obstacle_theta(theta1_sp, imp_theta_prev, imp_omega_prev, imp_theta_time_prev)
			imp_theta_prev = imp_theta
			imp_omega_prev = imp_omega
		imp_pose_from_theta = Obstacles_map[current_obstacle_index1] + np.array([R_obstacles*np.cos(imp_theta), R_obstacles*np.sin(imp_theta)])
		if theta_imp:
			drone1_pose_goal[:2] = imp_pose_from_theta
	# uncomment with real drone
	# else:
		# drone1_pose_goal += (drone1_pose - drone1_pose_goal) / 2.0



	# TO FLY
	if toFly:
		swarmlib.publish_goal_pos(drone1_pose_goal, 0, 'crazyflie1')
		swarmlib.publish_goal_pos(drone2_pose_goal, 0, 'crazyflie2')
		swarmlib.publish_goal_pos(drone3_pose_goal, 0, 'crazyflie3')

	# # TO VISUALIZE
	swarmlib.publish_pose(drone1_pose_goal, 0, "drone1_pose_goal")
	swarmlib.publish_pose(drone2_pose_goal, 0, "drone2_pose_goal")
	swarmlib.publish_pose(drone3_pose_goal, 0, "drone3_pose_goal")
	swarmlib.publish_pose(obstacle1_pose, 0, "obstacle1")
	swarmlib.publish_pose(obstacle2_pose, 0, "obstacle2")
	swarmlib.publish_pose(human_pose, 0, "human_pose")

	swarmlib.publish_path(path1, drone1_pose_goal, 0, cf1_name)
	swarmlib.publish_path(path2, drone2_pose_goal, 0, cf2_name)
	swarmlib.publish_path(path3, drone3_pose_goal, 0, cf3_name)


def follower():
	start_time = time.time()

	human_sub = message_filters.Subscriber('/vicon/'+human_name+'/'+human_name, TransformStamped)
	cf1_sub = message_filters.Subscriber('/vicon/'+cf1_name+'/'+cf1_name, TransformStamped)
	cf2_sub = message_filters.Subscriber('/vicon/'+cf2_name+'/'+cf2_name, TransformStamped)
	cf3_sub = message_filters.Subscriber('/vicon/'+cf3_name+'/'+cf3_name, TransformStamped)
	obstacle1_sub = message_filters.Subscriber('/vicon/obstacle1/obstacle1', TransformStamped)
	obstacle2_sub = message_filters.Subscriber('/vicon/obstacle2/obstacle2', TransformStamped)
	obstacle3_sub = message_filters.Subscriber('/vicon/obstacle3/obstacle3', TransformStamped)
	obstacle4_sub = message_filters.Subscriber('/vicon/obstacle4/obstacle4', TransformStamped)
	obstacle5_sub = message_filters.Subscriber('/vicon/obstacle5/obstacle5', TransformStamped)
	obstacle6_sub = message_filters.Subscriber('/vicon/obstacle6/obstacle6', TransformStamped)
	obstacle7_sub = message_filters.Subscriber('/vicon/obstacle7/obstacle7', TransformStamped)
	obstacle8_sub = message_filters.Subscriber('/vicon/obstacle8/obstacle8', TransformStamped)
	obstacle9_sub = message_filters.Subscriber('/vicon/obstacle9/obstacle9', TransformStamped)

	ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub,
																 cf2_sub,
																 cf3_sub,
																 		  obstacle1_sub,
																 		  obstacle2_sub,
																 		  # obstacle3_sub,
																 		  # obstacle4_sub,
																 		  # obstacle5_sub,
																 		  # obstacle6_sub,
																 		  # obstacle7_sub,
																 		  # obstacle8_sub,
																 		  # obstacle9_sub,
																 		  				], 10, 5)
	
	ts.registerCallback(tag_game)
	rospy.spin()


if __name__ == '__main__':
	rospy.init_node('follow_multiple', anonymous=True)

	if toFly:
		TAKEOFFHEIGHT = 0.2
		print "takeoff at "+str(TAKEOFFHEIGHT)
		cf1 = crazyflie.Crazyflie("crazyflie1", '/vicon/'+cf1_name+'/'+cf1_name)
		cf1.setParam("commander/enHighLevel", 1)
		cf1.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 1.0)
		cf2 = crazyflie.Crazyflie("crazyflie2", '/vicon/'+cf2_name+'/'+cf2_name)
		cf2.setParam("commander/enHighLevel", 1)
		cf2.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 1.0)
		cf3 = crazyflie.Crazyflie("crazyflie3", '/vicon/'+cf3_name+'/'+cf3_name)
		cf3.setParam("commander/enHighLevel", 1)
		cf3.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 1.0)
		time.sleep(2.0)



	print "\nfollowing human!\n"
	try:
		follower()
	except KeyboardInterrupt:
		pass

	# print "land"
	# cf1.land(targetHeight = 0.0, duration = 2.0)
	# cf2.land(targetHeight = 0.0, duration = 2.0)
	# cf3.land(targetHeight = 0.0, duration = 2.0)
	# time.sleep(2.0)


	print "Try to land"

	try:
		cf1.stop()
	except:
		pass
	try:
		cf2.stop()
	except:
		pass
	try:
		cf3.stop()
	except:
		pass
