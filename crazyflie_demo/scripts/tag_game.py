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
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})



# imp_pose_prev = 0
# imp_vel_prev = 0
# imp_time_prev = time.time()

imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

i = 0
updated_prev = False
updated11_prev = False


# def tag_game(human, cf1):
# def tag_game(human, cf1, cf2):
# def tag_game(human, cf1, cf2, cf3, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7):
def tag_game(human, cf1, cf2, cf3):
	print "tag_game"
	human_pose = swarmlib.get_coord(human)
	drone1_pose = swarmlib.get_coord(cf1)
	drone2_pose = swarmlib.get_coord(cf2)
	drone3_pose = swarmlib.get_coord(cf3)


	# obstacle1_pose = swarmlib.get_coord(obstacle1)
	# obstacle2_pose = swarmlib.get_coord(obstacle2)
	# obstacle3_pose = swarmlib.get_coord(obstacle3)
	# obstacle4_pose = swarmlib.get_coord(obstacle4)
	# obstacle5_pose = swarmlib.get_coord(obstacle5)
	# obstacle6_pose = swarmlib.get_coord(obstacle6)
	# obstacle7_pose = swarmlib.get_coord(obstacle7)

	l = 0.35

	hum_vel = swarmlib.hum_vel(human_pose)

	# HUMAN IMPEDANCE
	global imp_pose_prev
	global imp_vel_prev
	global imp_time_prev
	imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
	# print "imp_pose", imp_pose
	imp_pose_prev = imp_pose
	imp_vel_prev = imp_vel
	# swarmlib.publish_pose(human_pose + imp_pose, 0, "human_pose_imp")


	# all drones follow a human with impedance

	# drone1_pose_goal_x = (human_pose[0]-1.703)*1.920/0.900
	# -3*l
	drone1_pose_goal = np.array([  human_pose[0] -3.0*l        ,
								   human_pose[1]               ,
								   human_pose[2] - 0.1         ])
	drone2_pose_goal = np.array([ drone1_pose_goal[0] - l      ,
								  drone1_pose_goal[1] + l      ,
								  drone1_pose_goal[2]          ])
	drone3_pose_goal = np.array([ drone1_pose_goal[0] - l      ,
								  drone1_pose_goal[1] - l      ,
								  drone1_pose_goal[2]          ])
	# drone1_pose_goal = drone1_pose_goal + imp_pose
	drone2_pose_goal = drone2_pose_goal + imp_pose
	drone3_pose_goal = drone3_pose_goal + imp_pose
	drone2_pose_goal[1] = drone2_pose_goal[1] - imp_pose[0]*0.15
	drone3_pose_goal[1] = drone3_pose_goal[1] + imp_pose[0]*0.15


	# drone1_pose_goal, updated_1 = swarmlib.pose_update_obstacle(drone10_pose_goal, drone1_pose_goal)
	# drone2_pose_goal, updated_2 = swarmlib.pose_update_obstacle(drone10_pose_goal, drone2_pose_goal)
	# drone3_pose_goal, updated_3 = swarmlib.pose_update_obstacle(drone10_pose_goal, drone3_pose_goal)


	# TO FLY
	swarmlib.publish_goal_pos(drone1_pose_goal, 0, "/crazyflie13")
	swarmlib.publish_goal_pos(drone2_pose_goal, 0, "/crazyflie15")
	swarmlib.publish_goal_pos(drone3_pose_goal, 0, "/crazyflie18")

	# TO VISUALIZE
	swarmlib.publish_pose(drone1_pose_goal, 0, "drone1_pose_goal")
	swarmlib.publish_pose(drone2_pose_goal, 0, "drone2_pose_goal")
	swarmlib.publish_pose(drone3_pose_goal, 0, "drone3_pose_goal")
	# swarmlib.publish_pose(obstacle1_pose, 0, "obstacle1")
	# swarmlib.publish_pose(obstacle2_pose, 0, "obstacle2")
	# swarmlib.publish_pose(obstacle3_pose, 0, "obstacle3")
	# swarmlib.publish_pose(obstacle4_pose, 0, "obstacle4")
	# swarmlib.publish_pose(obstacle5_pose, 0, "obstacle5")
	# swarmlib.publish_pose(obstacle6_pose, 0, "obstacle6")
	# swarmlib.publish_pose(obstacle7_pose, 0, "obstacle7")
	swarmlib.publish_pose(human_pose, 0, "human_pose")




def follower():
	
	start_time = time.time()

	human_sub = message_filters.Subscriber('/vicon/human/human', TransformStamped)
	cf1_sub = message_filters.Subscriber('/vicon/crazyflie13/crazyflie13', TransformStamped)
	cf2_sub = message_filters.Subscriber('/vicon/crazyflie15/crazyflie15', TransformStamped)
	cf3_sub = message_filters.Subscriber('/vicon/crazyflie18/crazyflie18', TransformStamped)
	#cf4_sub = message_filters.Subscriber('/vicon/crazyflie4/crazyflie4', TransformStamped)

	#cf10_sub = message_filters.Subscriber('/vicon/crazyflie10/crazyflie10', TransformStamped)
	#cf11_sub = message_filters.Subscriber('/vicon/crazyflie11/crazyflie11', TransformStamped)

	# obstacle1_sub = message_filters.Subscriber('/vicon/obstacle1/obstacle1', TransformStamped)
	# obstacle2_sub = message_filters.Subscriber('/vicon/obstacle2/obstacle2', TransformStamped)
	# obstacle3_sub = message_filters.Subscriber('/vicon/obstacle3/obstacle3', TransformStamped)
	# obstacle4_sub = message_filters.Subscriber('/vicon/obstacle4/obstacle4', TransformStamped)
	# obstacle5_sub = message_filters.Subscriber('/vicon/obstacle5/obstacle5', TransformStamped)
	# obstacle6_sub = message_filters.Subscriber('/vicon/obstacle6/obstacle6', TransformStamped)
	# obstacle7_sub = message_filters.Subscriber('/vicon/obstacle7/obstacle7', TransformStamped)
	# ts = message_filters.TimeSynchronizer([human_sub, cf1_sub], 10)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, obstacle1_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, obstacle1_sub, obstacle2_sub, obstacle3_sub, obstacle4_sub, obstacle5_sub, obstacle6_sub, obstacle7_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, cf4_sub], 10, 5)
	ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub], 10, 5)

	ts.registerCallback(tag_game)
	rospy.spin()


if __name__ == '__main__':

	rospy.init_node('follow_multiple', anonymous=True)


	print "takeoff"
	cf1 = crazyflie.Crazyflie("crazyflie13", "/vicon/crazyflie13/crazyflie13")
	cf1.setParam("commander/enHighLevel", 1)
	cf1.takeoff(targetHeight = 1.2, duration = 2.0)
	cf2 = crazyflie.Crazyflie("crazyflie15", "/vicon/crazyflie15/crazyflie15")
	cf2.setParam("commander/enHighLevel", 1)
	cf2.takeoff(targetHeight = 1.2, duration = 2.0)
	cf3 = crazyflie.Crazyflie("crazyflie18", "/vicon/crazyflie18/crazyflie18")
	cf3.setParam("commander/enHighLevel", 1)
	cf3.takeoff(targetHeight = 1.2, duration = 2.0)

	time.sleep(3.0)


	print "\nfollowing human!\n"
	try:
		follower()
	except KeyboardInterrupt:
		pass


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

