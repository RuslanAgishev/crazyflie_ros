#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.msg import FullState
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path

from math import *
import time
import numpy as np

import crazyflie
import swarmlib

import sys
import os
from multiprocessing import Process

from numpy.linalg import norm
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path

from PathSmoothing import SmoothPath
from tools import *
from rrt_path_planner import rrt_path


def draw_map(obstacles, params):
    # Draw obstacles
    fig = plt.figure(figsize=(10, 10))
    plt.grid()
    ax = plt.gca()
    ax.set_xlim(params.world_bounds_x)
    ax.set_ylim(params.world_bounds_y)
    for k in range(len(obstacles)):
        ax.add_patch( Polygon(obstacles[k]) )

class RRT_Params:
    def __init__(self):
		self.toFly   = 0
		self.animate = 0 # show RRT construction, set 0 to reduce time of the RRT algorithm
		self.visualize = 1 # show constructed paths at the end of the RRT and path smoothing algorithms
		self.maxiters = 5000 # max number of samples to build the RRT
		self.goal_prob = 0.05 # with probability goal_prob, sample the goal
		self.minDistGoal = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
		self.extension = 0.2 # [m], extension parameter: this controls how far the RRT extends in each step.
		self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
		self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
		self.TakeoffHeight  = 2.0 # meters
		self.TakeoffTime    = 5.0 # seconds
		self.ViconRate      = 200 # [Hz]
		self.drone_vel      = 0.4 # [m/s]

def waypts2setpts(P, params):
	"""
	construct a long array of setpoints, traj_global, with equal inter-distances, dx,
	from a set of via-waypoints, P = [[x0,y0], [x1,y1], ..., [xn,yn]]
	"""
	V = params.drone_vel # [m/s]
	freq = params.ViconRate; dt = 1./freq
	dx = V * dt
	traj_global = np.array(P[-1])
	for i in range(len(P)-1, 0, -1):
		A = P[i]
		B = P[i-1]

		n = (B-A) / norm(B-A)
		delta = n * dx
		N = int( norm(B-A) / norm(delta) )
		sp = A
		traj_global = np.vstack([traj_global, sp])
		for i in range(N):
			sp += delta
			traj_global = np.vstack([traj_global, sp])
		sp = B
		traj_global = np.vstack([traj_global, sp])

	return traj_global


if __name__ == '__main__':
	rospy.init_node('rrt_planner', anonymous=True)

	# Initialization
	params = RRT_Params()
	rate = rospy.Rate(params.ViconRate)

	# Obstacles. An obstacle is represented as a convex hull of a number of points. 
	# First row is x, second is y (position of vertices)
	w = 0.2
	obstacles = [
              np.array([[0, -1], [1, -1], [1, -0.9], [0, w-1]]),
              np.array([[0, -1], [w, -0.8], [0.1, 1], [0.0, 1.0]]),
              np.array([[0, 1-w], [1, 1], [1, 1+w], [0, 1+w]]),
              np.array([[1-w, -1], [1+w, -1], [1+w, 0], [1, 0]]),
              np.array([[1-w, 1+w], [1+w, 1+w], [1+w, 0.5], [1, 0.5]]),
              np.array([[0.8, 0], [1+w, 0], [1+w, w], [0.8, w]]),
              np.array([[0.8, 0.5], [1+w, 0.5], [1+w, 0.5+w], [0.8, 0.5+w]]),

              # np.array([[-0.5, -0.5], [-1.5, -0.5], [-1-w, -1.5-w], [-0.8, -1.5-w]]),

              np.array([[-0.5, 1.2], [-2.0, 1.2], [-1-w, 1.5+w], [-0.8, 1.5+w]])
            ]

	draw_map(obstacles, params)

	# DRONES INIT
	# cf_names         = ['cf1',
	# 					'cf2',
	# 					'cf3',
	# 					'cf4'
	# 				   ]
	cf_names = ['cf1']
	drones = []
	for name in cf_names:
		drones.append( swarmlib.Drone(name) )
		drones[-1].sp = drones[-1].position()

	# Start and goal positions
	# xy_start = np.array([0.5, 0.5])
	xy_start = drones[0].position()[:2]  
	xy_goal =  np.array([-0.8, -0.2])

	plt.plot(xy_start[0], xy_start[1],'bo',color='red', markersize=20)
	plt.plot(xy_goal[0],  xy_goal[1], 'bo',color='green',markersize=20)

	P = rrt_path(obstacles, xy_start, xy_goal, params)
	print 'Path smoothing...'
	P_smooth = SmoothPath(P, obstacles, smoothiters=40)
	plt.plot(P_smooth[:,0], P_smooth[:,1], linewidth=5, color='orange', label='smoothed path')
	plt.legend()

	if params.visualize: plt.show()


	if params.toFly:
		cf_list = []
		print "Takeoff"
		for name in cf_names:
		    cf = crazyflie.Crazyflie(name, '/vicon/'+name+'/'+name)
		    cf.setParam("commander/enHighLevel", 1)
		    cf.setParam("stabilizer/estimator", 2)  # Use EKF
		    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
		    cf.takeoff(targetHeight = params.TakeoffHeight, duration = params.TakeoffTime)
		    cf_list.append(cf)
		time.sleep(params.TakeoffTime)
		# for cf in cf_list: cf.land(targetHeight = -0.1, duration = 3.0)
		# time.sleep(3.0)
		# for cf in cf_list: cf.stop()



	""" trajectory generation """
	traj_global = waypts2setpts(P_smooth, params)

	""" main loop for setpoints sending to robots """
	print "Flight..."
	i = 0
	while i < traj_global.shape[0] and not rospy.is_shutdown():
		sp = traj_global[i,:]
		
		# TODO: correct the trajectory from global global planner
		# with local planner (Potential Fields).


		drones[0].sp = np.hstack([sp, params.TakeoffHeight])

		if params.toFly:
			for drone in drones: drone.fly()

		# TO VISUALIZE
		for drone in drones:
			drone.publish_sp()
			drone.publish_path_sp(limit=-1)
		i += 1
		rate.sleep()




	print 'Landing!!!'
	while not rospy.is_shutdown():
		for drone in drones:
			drone.sp[2] -= 0.0015
		if params.toFly:
			for drone in drones: drone.fly()
		for drone in drones:
			drone.publish_sp()
			drone.publish_path_sp()

		if drones[0].sp[2]<-0.5: #and drones[1].sp[2]<-1.0 and drones[2].sp[2]<-1.0 and drones[3].sp[2]<-1.0:
			time.sleep(0.3)
			if params.toFly:
				for cf in cf_list: cf.stop()
			print 'reached the floor, shutdown'
			rospy.signal_shutdown('landed')
		rate.sleep()