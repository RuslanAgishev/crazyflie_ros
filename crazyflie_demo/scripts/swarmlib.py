#!/usr/bin/env 

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from tf import TransformListener

from scipy.integrate import odeint
from scipy.spatial.distance import cdist
from math import *
import math

import time
from time import sleep

import message_filters
import sys
import numpy as np

from multiprocessing import Process
import os

from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position

import serial
import matplotlib.pyplot as plt

# np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

class Centroid:
	"""Centroid of the Swarm object"""
	def __init__(self, obstacles, name='centroid'):
		self.name = name
		self.tf = '/vicon/'+self.name+'/'+self.name
		self.pose = np.array([-5, -5, 0])
		self.orient = np.array([0,0,0])
		self.sp = self.pose
		self.obstacles = obstacles
		self.path = Path()
		self.delta = np.array([0,0])
		self.near_obstacle = np.zeros(len(obstacles))
		self.radius_impedance = Radius_Impedance()
		self.angular_impedance = Angular_Impedance()
		self.impedance_avel = Impedance_avel()
		self.theta = None
		self.d_theta = pi*np.ones(len(obstacles))
		self.dist_to_drones = np.zeros(3)
		self.dist_to_obst = np.zeros(len(obstacles))
		self.drone_time_array = np.ones(10)
		self.drone_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
		self.rate = rospy.Rate(100)
		self.traj = self.pose
		self.feature = self.sp
		self.start = self.sp
		self.goal  = self.sp
	
	def publish_sp(self):
	    publish_pose(self.sp, np.array([0,0,0]), self.name+"_sp")

	def publish_position(self):
	    publish_pose(self.pose, self.orient, self.name+"_pose")

	def publish_path(self, limit=1000):
		publish_path(self.path, self.sp, self.orient, self.name+"_path", limit)
		#for i in range( 1,len(self.path.poses) ):
		#	print self.name +": "+ str(self.path.poses[i].pose)

	def update_pose_theta(self, drone, obstacle, R):
		drone_omega = self.omegaZ(drone.sp, drone.sp-obstacle.position())
		if drone.near_obstacle[obstacle.id] == 1: # initial impedance parameters, when the drone is just near the obstacle
			drone.angular_impedance.imp_theta, drone.angular_impedance.imp_omega, drone.angular_impedance.imp_time =\
				drone.angular_impedance.impedance_model(drone.theta, drone.theta, drone_omega, drone.angular_impedance.imp_time)
		else:
			drone.angular_impedance.imp_theta, drone.angular_impedance.imp_omega, drone.angular_impedance.imp_time =\
				drone.angular_impedance.impedance_model(drone.theta, drone.angular_impedance.imp_theta, drone.angular_impedance.imp_omega, drone.angular_impedance.imp_time)
		# print('w=', drone.angular_impedance.imp_omega)

		updated_pose = drone.angular_impedance.pose_from_theta( obstacle.position()[:2], R, drone.angular_impedance.imp_theta )
		return updated_pose

	def update_pose_radius(self, pose_prev, drone, dist_to_obstacle, koef):
		R = dist_to_obstacle
		drone_vel = drone.velocity(drone.sp)
		drone_vel_n = np.dot(drone_vel, R)/(np.linalg.norm(R)**2) * R
		drone_vel_t = drone_vel - drone_vel_n
		# print('v_n', drone_vel_n[:2])
		# print('v_t', drone_vel_t[:2])
		if (sum(drone.near_obstacle) <= 10): # 10 is the number of samples to compute velocity from consequent poses
			drone.radius_impedance.imp_vel = np.hstack((drone_vel_n, 0))
			# drone.radius_impedance.imp_vel = drone_vel
		drone.radius_impedance.imp_pose, \
		drone.radius_impedance.imp_vel, \
		drone.radius_impedance.imp_time = \
			drone.radius_impedance.impedance_model(drone.delta, drone.radius_impedance.imp_pose, drone.radius_impedance.imp_vel, drone.radius_impedance.imp_time)
		delta_pose = drone.radius_impedance.imp_pose[:2]
		updated_pose = pose_prev + koef*delta_pose

		''' impedance model vizualization '''
		length = np.linalg.norm(delta_pose)
		yaw = acos(delta_pose[0] / length)
		publish_arrow(np.hstack([updated_pose,drone.sp[2]]), [0,0,yaw], length,  '/delta_pose')

		return updated_pose

	def calculate_dist_to_drones(self, drones_poses):
		for i in range(len(drones_poses)):
			self.dist_to_drones[i] = np.linalg.norm(drones_poses[i] - self.sp)

	def dist_to_obstacles(self):
		dist = []
		for i in range(len(self.obstacles)):
			dist.append( np.linalg.norm(self.obstacles[i].position()[:2]-self.sp[:2]) )
		self.dist_to_obst = dist
		return np.array(dist)

	def omegaZ(self, drone_pose, dist_from_obstacle):
		R = dist_from_obstacle
		drone_vel = self.velocity(drone_pose)
		# drone_vel_n = np.dot(drone_vel, R)/(np.linalg.norm(R)**2) * R
		# drone_vel_t = drone_vel - drone_vel_n
		
		drone_w = np.cross(R, drone_vel) / ( np.linalg.norm(R)**2 )

		return drone_w[2]

	def velocity(self, drone_pose):
		for i in range(len(self.drone_time_array)-1):
			self.drone_time_array[i] = self.drone_time_array[i+1]
		self.drone_time_array[-1] = time.time()

		for i in range(len(self.drone_pose_array[0])-1):
			self.drone_pose_array[0][i] = self.drone_pose_array[0][i+1]
			self.drone_pose_array[1][i] = self.drone_pose_array[1][i+1]
			self.drone_pose_array[2][i] = self.drone_pose_array[2][i+1]
		self.drone_pose_array[0][-1] = drone_pose[0]
		self.drone_pose_array[1][-1] = drone_pose[1]
		self.drone_pose_array[2][-1] = drone_pose[2]

		vel_x = (self.drone_pose_array[0][-1]-self.drone_pose_array[0][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])
		vel_y = (self.drone_pose_array[1][-1]-self.drone_pose_array[1][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])
		vel_z = (self.drone_pose_array[2][-1]-self.drone_pose_array[2][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])

		drone_vel = np.array( [vel_x, vel_y, vel_z] )

		return drone_vel


class Drone:
	def __init__(self, name, obstacles, leader = False):
		self.name = name
		self.tf = '/vicon/'+name+'/'+name
		self.leader = leader
		self.tl = TransformListener()
		self.pose = self.position()
		self.orient = np.array([0,0,0])
		self.sp = self.position()
		self.obstacles = obstacles
		self.path = Path()
		self.delta = np.array([0,0])
		self.near_obstacle = np.zeros(len(obstacles))
		self.radius_impedance = Radius_Impedance()
		self.angular_impedance = Angular_Impedance()
		self.impedance_avel = Impedance_avel()
		self.theta = None
		self.d_theta = pi*np.ones(len(obstacles))
		self.dist_to_drones = np.zeros(3)
		self.dist_to_obst = np.zeros(len(obstacles))
		self.drone_time_array = np.ones(10)
		self.drone_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
		self.rate = rospy.Rate(100)
		self.traj = np.array([0,0,0])
		self.feature = self.sp
		self.start = self.position()
		self.goal  = self.sp

	def position(self):
	    self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
	    position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
	    self.pose = position
	    return np.array(position)

	def orientation(self):
	    self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
	    position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
	    self.orient = get_angles(np.array(quaternion))
	    return get_angles(np.array(quaternion))

	def publish_sp(self):
	    publish_pose(self.sp, np.array([0,0,0]), self.name+"_sp")

	def publish_position(self):
	    publish_pose(self.pose, self.orient, self.name+"_pose")

	def publish_path(self, limit=1000):
		publish_path(self.path, self.sp, self.orient, self.name+"_path", limit)
		#for i in range( 1,len(self.path.poses) ):
		#	print self.name +": "+ str(self.path.poses[i].pose)

	def fly(self):
		# if self.leader:
		# 	limits = np.array([ 2, 2, 2.5 ])
		# 	np.putmask(self.sp, self.sp >= limits, limits)
		# 	np.putmask(self.sp, self.sp <= -limits, -limits)
		publish_goal_pos(self.sp, 0, "/"+self.name)

	def update_pose_theta(self, drone, obstacle, R):
		drone_omega = self.omegaZ(drone.sp, drone.sp-obstacle.position())
		if drone.near_obstacle[obstacle.id] == 1: # initial impedance parameters, when the drone is just near the obstacle
			drone.angular_impedance.imp_theta, drone.angular_impedance.imp_omega, drone.angular_impedance.imp_time =\
				drone.angular_impedance.impedance_model(drone.theta, drone.theta, drone_omega, drone.angular_impedance.imp_time)
		else:
			drone.angular_impedance.imp_theta, drone.angular_impedance.imp_omega, drone.angular_impedance.imp_time =\
				drone.angular_impedance.impedance_model(drone.theta, drone.angular_impedance.imp_theta, drone.angular_impedance.imp_omega, drone.angular_impedance.imp_time)

		updated_pose = drone.angular_impedance.pose_from_theta( obstacle.position()[:2], R, drone.angular_impedance.imp_theta )
		return updated_pose

	def update_pose_radius(self, pose_prev, drone, dist_to_obstacle, koef):
		if (sum(drone.near_obstacle) <= 10): # 10 is the number of samples to compute velocity from consequent poses
			drone.radius_impedance.imp_vel = drone.velocity(drone.sp)
		drone.radius_impedance.imp_pose, \
		drone.radius_impedance.imp_vel, \
		drone.radius_impedance.imp_time = \
			drone.radius_impedance.impedance_model(drone.delta, drone.radius_impedance.imp_pose, drone.radius_impedance.imp_vel, drone.radius_impedance.imp_time)
		delta_pose = drone.radius_impedance.imp_pose[:2]
		updated_pose = pose_prev + koef*delta_pose

		''' impedance model vizualization '''
		length = np.linalg.norm(delta_pose)
		yaw = acos(delta_pose[0] / length)
		publish_arrow(np.hstack([updated_pose,drone.sp[2]]), [0,0,yaw], length,  '/delta_pose')

		return updated_pose

	def update_pose_avel(self, pose_prev, average_vel):
		self.impedance_avel.imp_pose, self.impedance_avel.imp_vel, self.impedance_avel.imp_time = \
			self.impedance_avel.impedance_model(average_vel, self.impedance_avel.imp_pose, self.impedance_avel.imp_vel, self.impedance_avel.imp_time)
		updated_pose = pose_prev + 0.15*self.impedance_avel.imp_pose
		return updated_pose

	def calculate_dist_to_drones(self, drones_poses):
		for i in range(len(drones_poses)):
			self.dist_to_drones[i] = np.linalg.norm(drones_poses[i] - self.sp)

	def dist_to_obstacles(self):
		dist = []
		for i in range(len(self.obstacles)):
			dist.append( np.linalg.norm(self.obstacles[i].position()[:2]-self.sp[:2]) )
		self.dist_to_obst = dist
		return np.array(dist)

	def omegaZ(self, drone_pose, dist_from_obstacle):
		R = dist_from_obstacle # 3D-vector
		drone_vel = self.velocity(drone_pose)
		# drone_vel_n = np.dot(drone_vel, R)/(np.linalg.norm(R)**2) * R
		# drone_vel_t = drone_vel - drone_vel_n
		drone_w = np.cross(R, drone_vel) # / ( np.linalg.norm(R)**2 )

		return drone_w[2]

	def velocity(self, drone_pose):
		for i in range(len(self.drone_time_array)-1):
			self.drone_time_array[i] = self.drone_time_array[i+1]
		self.drone_time_array[-1] = time.time()

		for i in range(len(self.drone_pose_array[0])-1):
			self.drone_pose_array[0][i] = self.drone_pose_array[0][i+1]
			self.drone_pose_array[1][i] = self.drone_pose_array[1][i+1]
			self.drone_pose_array[2][i] = self.drone_pose_array[2][i+1]
		self.drone_pose_array[0][-1] = drone_pose[0]
		self.drone_pose_array[1][-1] = drone_pose[1]
		self.drone_pose_array[2][-1] = drone_pose[2]

		vel_x = (self.drone_pose_array[0][-1]-self.drone_pose_array[0][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])
		vel_y = (self.drone_pose_array[1][-1]-self.drone_pose_array[1][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])
		vel_z = (self.drone_pose_array[2][-1]-self.drone_pose_array[2][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])

		drone_vel = np.array( [vel_x, vel_y, vel_z] )

		return drone_vel

	def landing(self, sim=False):
		drone_landing_pose = self.position() if sim==False else self.sp
		while not rospy.is_shutdown():
			self.sp = drone_landing_pose
			drone_landing_pose[2] = drone_landing_pose[2]-0.007
			self.publish_sp()
			self.publish_path(limit=1000)
			if sim==False:
				self.fly()
			if self.sp[2]<-1.0:
				sleep(1)
				if sim==False:
					cf1.stop()
				print 'reached the floor, shutdown'
				# rospy.signal_shutdown('landed')
			self.rate.sleep()


class Mocap_object:
    def __init__(self, name):
        self.name = name
        self.tf = '/vicon/'+name+'/'+name
        self.tl = TransformListener()
        self.pose = np.array([0,0,0])
        self.orient = np.array([0,0,0])
        self.angles_to_drones = None

    def position(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.pose = position
        return np.array(position)

    def orientation(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.orient = get_angles(np.array(quaternion))
        return get_angles(np.array(quaternion))

    def publish_position(self):
        publish_pose(self.pose, self.orient, self.name+"_pose")


class Obstacle:
    def __init__(self, name, id, R_obstacle):
        self.name = name
        self.id = id
        self.R = R_obstacle
        self.tf = '/vicon/'+name+'/'+name
        self.tl = TransformListener()
        self.pose = self.position()
        self.orient = np.array([0,0,0])
        self.dist_to_drones = np.zeros(3)

    def position(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.pose = position
        return np.array(position)

    def orientation(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.orient = get_angles(np.array(quaternion))
        return get_angles(np.array(quaternion))

    def publish_position(self):
        # publish_pose(self.pose, self.orient, self.name+"_pose")
        publish_cylinder(self.pose, self.orient, self.R, self.name+"_cylinder")

    def calculate_dist(self, drones_poses):
    	for i in range(len(drones_poses)):
        	self.dist_to_drones[i] = np.linalg.norm(drones_poses[i]-self.pose)

    def circle_points(self, R, N=1000):
    	C = np.zeros((N,2))
    	# C[0,:] = self.position()[0] + self.R*np.cos(np.linspace(-pi,pi,N))
    	# C[1,:] = self.position()[1] + self.R*np.sin(np.linspace(-pi,pi,N))
    	C[:,0] = self.pose[0] + R*np.cos(np.linspace(-pi,pi,N))
    	C[:,1] = self.pose[1] + R*np.sin(np.linspace(-pi,pi,N))
    	return C


class Radius_Impedance:
	def __init__(self):
		self.imp_pose = np.array([0,0,0])
		self.imp_vel = np.array([0,0,0])
		self.imp_time = time.time()

	def impedance_model(self, delta, imp_pose_prev, imp_vel_prev, time_prev):
		F_coeff = 12 # 7
		time_step = time.time() - time_prev
		time_prev = time.time()
		t = [0. , time_step]
		F =  - delta * F_coeff

		state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
		state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
		state_x = state_x[1]

		state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
		state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
		state_y = state_y[1]

		imp_pose = np.array( [state_x[0], state_y[0], 0] )
		imp_vel  = np.array( [state_x[1], state_y[1], 0] )

		return imp_pose, imp_vel, time_prev


class Angular_Impedance:
	def __init__(self):
		self.imp_pose = None
		self.imp_theta = 0
		self.imp_omega = 0
		self.imp_time = time.time()

	def impedance_model(self, theta, imp_theta_prev, imp_omega_prev, time_prev):
		M_coeff = 10 # 7
		time_step = time.time() - time_prev
		time_prev = time.time()
		t = [0. , time_step]
		M = - sin(imp_theta_prev - theta) * M_coeff
		state0 = [imp_theta_prev, imp_omega_prev]
		state = odeint(Pendulum, state0, t, args=(M,))
		state = state[1]

		imp_theta = state[0]
		imp_omega = state[1]
		return imp_theta, imp_omega, time_prev

	def pose_from_theta(self, obstacle_pose, R, theta):
		return obstacle_pose + np.array([R*np.cos(theta), R*np.sin(theta)])

	# theta_from_pose returns angle between 2 vectors: X and [drone_pose-obstacle_pose]' in XY-plane
	def theta_from_pose(self, object1, object2):
		theta = np.sign(object1[1]-object2[1]) * acos( (object1[0]-object2[0]) / np.linalg.norm(object1[:2] - object2[:2]) ) # [-pi,pi] - range
		return theta

class Impedance_avel:
	def __init__(self):
		self.imp_pose = np.array([0,0,0])
		self.imp_vel = np.array([0,0,0])
		self.imp_time = time.time()

	def impedance_model(self, delta, imp_pose_prev, imp_vel_prev, time_prev):
		F_coeff = 12 # 7
		time_step = time.time() - time_prev
		time_prev = time.time()
		t = [0. , time_step]
		F = - delta * F_coeff

		state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
		state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
		state_x = state_x[1]

		state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
		state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
		state_y = state_y[1]

		imp_pose = np.array( [state_x[0], state_y[0], 0] )
		imp_vel  = np.array( [state_x[1], state_y[1], 0] )
		return imp_pose, imp_vel, time_prev


def intersection(curve1, curve2):
	dists = cdist(curve1, curve2)
	i1, i2 = np.where(dists==np.min(dists))
	i1 = i1[0]; i2 = i2[0]
	return np.array([curve1[i1,0], curve1[i1,1]])


# serial_port = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_956353330313512012D0-if00', 9600)
def send_velocity(pattern):
    item = '%s\r' % pattern
    serial_port.write(item.encode())
    time.sleep(0.025)

def tactile_patterns(drone1_pose_goal, drone2_pose_goal, drone3_pose_goal, prev_pattern_time):
		# ############ TACTILE PATTERNS ##########################

		# AREA calc
		# https://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates
		x = np.array([drone1_pose_goal[0], drone2_pose_goal[0], drone3_pose_goal[0]])
		y = np.array([drone1_pose_goal[1], drone2_pose_goal[1], drone3_pose_goal[1]])
		def PolyArea(x,y):
			return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))
		# print 'area', PolyArea(x,y)
		# if PolyArea(x,y)<0.4:


		if (time.time()-prev_pattern_time)>1.5:

			if PolyArea(x,y)>0.075:
				print "extended, area = ", PolyArea(x,y)
				extended_pattern()
				# swarmlib.send_velocity(6)
				prev_pattern_time = time.time()

			elif PolyArea(x,y)<0.04:
				print "contracted, area = ", PolyArea(x,y)
				# swarmlib.send_velocity(3)
				# contracted_pattern()
				prev_pattern_time = time.time()

		return prev_pattern_time
		# #############################################################3


def msg_def_crazyflie(pose, yaw):
	worldFrame = rospy.get_param("~worldFrame", "/world")
	msg = Position()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.x = pose[0]
	msg.y = pose[1]
	msg.z = pose[2]
	msg.yaw = yaw
	now = rospy.get_time()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	return msg

def msg_def_PoseStamped(pose, orient):
	worldFrame = "world"
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.header.seq += 1
	msg.header.stamp = rospy.Time.now()
	return msg

def msg_def_Cylinder(pose, orient, shape, R):
	worldFrame = "world"
	msg = Marker()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.type = shape
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2] * 0.5
	# quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2])
	quaternion = tf.transformations.quaternion_from_euler(0,0,0)
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.scale.x = R
	msg.scale.y = R
	msg.scale.z = 2.0
	msg.color.r = 0.0
	msg.color.g = 1.0
	msg.color.b = 0.0
	msg.color.a = 1.0
	msg.header.seq += 1
	msg.header.stamp = rospy.Time.now()
	return msg

def msg_def_Arrow(pose, orient, length):
	arrow = Marker()
	arrow.header.frame_id = "world"
	arrow.header.stamp = rospy.Time.now()
	arrow.type = arrow.ARROW
	arrow.pose.position.x = pose[0]
	arrow.pose.position.y = pose[1]
	arrow.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
	arrow.pose.orientation.x = quaternion[0]
	arrow.pose.orientation.y = quaternion[1]
	arrow.pose.orientation.z = quaternion[2]
	arrow.pose.orientation.w = quaternion[3]
	arrow.scale.x = length
	arrow.scale.y = 0.02
	arrow.scale.z = 0.02
	arrow.color.a = 1.0
	arrow.color.r = 1.0
	arrow.color.g = 0.5 
	arrow.color.b = 0.5
	arrow.header.seq += 1
	arrow.header.stamp = rospy.Time.now()
	return arrow

def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
	name = cf_name + "/cmd_position"
	msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
	pub = rospy.Publisher(name, Position, queue_size=1)
	pub.publish(msg)

def get_coord(PoseStamped_message):
	x = PoseStamped_message.transform.translation.x
	y = PoseStamped_message.transform.translation.y
	z = PoseStamped_message.transform.translation.z
	coord_array = np.array([x, y, z])
	return coord_array

def get_angles(message):
	quat = ( message[0], message[1], message[2], message[3] )
	euler = tf.transformations.euler_from_quaternion(quat)
	return euler

def publish_pose(pose, orient, topic_name):
	msg = msg_def_PoseStamped(pose, orient)
	pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	pub.publish(msg)

def publish_path(path, pose, orient, topic_name, limit=1000):
	msg = msg_def_PoseStamped(pose, orient)
	path.header = msg.header
	path.poses.append(msg)
	if limit>0:
		path.poses = path.poses[-limit:]
	pub = rospy.Publisher(topic_name, Path, queue_size=1)
	pub.publish(path)

def publish_cylinder(pose, orient, R, topic_name):
	shape = Marker.CYLINDER
	msg = msg_def_Cylinder(pose, orient, shape, R=R)
	pub = rospy.Publisher(topic_name, Marker, queue_size=1)
	pub.publish(msg)

def publish_arrow(pose, orient, length, topic_name):
	shape = Marker.ARROW
	msg = msg_def_Arrow(pose, orient, length)
	pub = rospy.Publisher(topic_name, Marker, queue_size=1)
	pub.publish(msg)


# HUMAN VELOCITY CALCULATION
hum_time_array = np.ones(10)
hum_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
def hum_vel(human_pose):

	for i in range(len(hum_time_array)-1):
		hum_time_array[i] = hum_time_array[i+1]
	hum_time_array[-1] = time.time()

	for i in range(len(hum_pose_array[0])-1):
		hum_pose_array[0][i] = hum_pose_array[0][i+1]
		hum_pose_array[1][i] = hum_pose_array[1][i+1]
		hum_pose_array[2][i] = hum_pose_array[2][i+1]
	hum_pose_array[0][-1] = human_pose[0]
	hum_pose_array[1][-1] = human_pose[1]
	hum_pose_array[2][-1] = human_pose[2]

	vel_x = (hum_pose_array[0][-1]-hum_pose_array[0][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_y = (hum_pose_array[1][-1]-hum_pose_array[1][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_z = (hum_pose_array[2][-1]-hum_pose_array[2][0])/(hum_time_array[-1]-hum_time_array[0])

	hum_vel = np.array( [vel_x, vel_y, vel_z] )

	return hum_vel

# HUMAN IMPEDANCE
def MassSpringDamper(state,t,F):
	x = state[0]
	xd = state[1]
	m = 2.0 # Kilograms
	b = 12.6
	k = 20.0 # Newtons per meter
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]

def Pendulum(state, t, M):
    theta, omega = state
    J = 1.; b = 10.; k = 0.
    dydt = [omega, (M - b*omega - k*np.sin(theta)) / J ]
    return dydt

def impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, time_prev):
	F_coeff = 12 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	F = - hum_vel * F_coeff

	state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
	state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
	state_x = state_x[1]

	state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
	state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
	state_y = state_y[1]

	state0_z = [imp_pose_prev[2], imp_vel_prev[2]]
	state_z = odeint(MassSpringDamper, state0_z, t, args=(F[2],))
	state_z = state_z[1]

	imp_pose = np.array( [state_x[0], state_y[0], state_z[0]] )
	imp_vel  = np.array( [state_x[1], state_y[1], state_z[1]] )

	# return state[0]
	return imp_pose, imp_vel, time_prev


def pub_circle_traj(x0,y0,z0,r,i):
	# i=0
	# while time_delay<delay:
		
	x1 = x0 + r*sin(i*1.75*pi/360) # 1
	y1 = y0 + r*cos(i*1.75*pi/360) # 1
	z1 = z0

	drone10_pose_goal = np.array([ x1,y1,z1 ])

	x2 = x0 + r*sin(i*1.75*pi/360+pi) # 2
	y2 = y0 + r*cos(i*1.75*pi/360+pi) # 2
	z2 = z0
	
	drone11_pose_goal = np.array([ x2,y2,z2 ])

	i = i+1
	
	# publish_goal_pos(drone10_pose_goal, 0, "/crazyflie10")
	# publish_goal_pos(drone11_pose_goal, 0, "/crazyflie11")

	publish_pose(drone10_pose_goal, 0, "drone10_pose_goal")
	publish_pose(drone11_pose_goal, 0, "drone11_pose_goal")

	return i, drone10_pose_goal, drone11_pose_goal

def rotate(origin, drone, human):
	"""
	Rotate a point counterclockwise by a given angle around a given origin.
	The angle should be given in radians.
	"""
	ox, oy = origin[0], origin[1]
	px, py = drone.sp[0], drone.sp[1]

	qx = ox + math.cos(human.orientation()[2]) * (px - ox) - math.sin(human.orientation()[2]) * (py - oy)
	qy = oy + math.sin(human.orientation()[2]) * (px - ox) + math.cos(human.orientation()[2]) * (py - oy)
	
	return np.array([qx, qy, drone.sp[2]])

def centroid_calc(drone1, drone2, drone3):
	x_aver = np.array([drone1.sp[0], drone2.sp[0], drone3.sp[0]])
	y_aver = np.array([drone1.sp[1], drone2.sp[1], drone3.sp[1]])
	z_aver = np.array([drone1.sp[2], drone2.sp[2], drone3.sp[2]])
	centroid = np.array([ np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
	return centroid






def extended_pattern():
	# 1st column is intensity levels between 0-9
	#2nd column is timing between 0-999
	time = 200
	#examples
	A = np.zeros((5, 1, 2))
	A = (
	[0, time],
	[0, time],
	[9, time],
	[0, time],
	[0, time])

	B = np.zeros((5, 1, 2))
	B = (
	[0, time],
	[9, time],
	[0, time],
	[9, time],
	[0, time])

	C = np.zeros((5, 1, 2))
	C = (
	[9, time],
	[0, time],
	[0, time],
	[0, time],
	[9, time])

	P= []
	P.append(A)
	P.append(B)
	P.append(C)
	# P.append(A)

	matrix_send(P)
def startXbee():
	serial_port = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_956353330313512012D0-if00', 9600)
def send_velocity(pattern):
    item = '%s\r' % pattern
    serial_port.write(item.encode())
def matrix_send(matrix):
    X = np.zeros((5, 1, 2))
    X = (
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0])
    for i in range(len(matrix)):
        matrix_data(matrix[i])
    for i in range (5- len(matrix)):
        matrix_data(X)
def matrix_data(Z):
    for k in range(len(Z)):
        send_velocity(Z[k][0])
        # time.sleep(0.05);
        # print("raw1", Z[k][0])
    cnt = 0
    a = 0
    c = 0

    for k in range(len(Z)):
        cnt = 0
        while (4 - len(str(Z[k][1])) - cnt):
            send_velocity(0)
            # time.sleep(0.05);
            # print("raw02", 0)
            cnt = cnt + 1
        for i in range(len(str(Z[k][1]))):
            c = Z[k][1] // 10 ** (len(str(Z[k][1])) - 1)
            send_velocity(c)
            # time.sleep(0.05);
            # print("raw2", c)
            Z[k][1] = Z[k][1] - c * 10 ** (len(str(Z[k][1])) - 1)


def pose_update_drone(drone, drone_obstacle, R):
	# drone avoids drone_obstacle on a circle trajectory if they are near
	# else there is an impedance between them based on their average speed
	obstacle_pose = drone_obstacle.sp[:2]
	average_vel = ( drone.velocity(drone.sp) + drone_obstacle.velocity(drone_obstacle.sp) ) / 2.0

	drone_pose = drone.sp[:2]
	dist = np.linalg.norm(obstacle_pose-drone_pose)
	if dist<R:
		updated_pose = quad_prog_circle(drone_pose, obstacle_pose, R)
		pose_is_updated = True
		updated_pose = np.append(updated_pose, drone.sp[2])
	else:
		updated_pose = drone.update_pose_avel(drone.sp, average_vel)
	 	pose_is_updated = False
	return updated_pose, pose_is_updated

def pose_update_obstacle(drone, obstacle, R, rad_imp=False, rad_imp_koef=0.3):
	obstacle_pose = obstacle.position()[:2]
	drone_pose = drone.sp[:2]
	dist = np.linalg.norm(obstacle_pose-drone_pose)
	if dist<R:
		updated_pose = quad_prog_circle(drone_pose, obstacle_pose, R)
		pose_is_updated = True
	else:
		updated_pose = drone_pose
		pose_is_updated = False

	if rad_imp:
		drone.delta = updated_pose - drone_pose
		if np.linalg.norm(drone.delta) > 0:
			updated_pose = drone.update_pose_radius(updated_pose, drone, obstacle.position()-drone.sp, koef=rad_imp_koef)
			# updated_pose -= rad_imp_koef*drone.delta

	updated_pose = np.append(updated_pose, drone.sp[2])

	return updated_pose, pose_is_updated


def pose_update_obstacle_imp(drone, obstacle, R, rad_imp=False, rad_imp_koef=0.15):
	obstacle_pose = obstacle.position()[:2]
	drone_pose = drone.sp[:2]	
	drone.theta = drone.angular_impedance.theta_from_pose(drone_pose, obstacle.pose)
	dist = np.linalg.norm(obstacle_pose-drone_pose)
	# if drone.dist_to_obstacles()[obstacle.id]<R: #or abs(drone.d_theta[obstacle.id])>pi/16.:
	if dist < R:
		drone.near_obstacle[obstacle.id] += 1
		updated_pose = drone.update_pose_theta(drone, obstacle, R)
		drone.d_theta[obstacle.id] = drone.angular_impedance.theta_from_pose(drone_pose, obstacle_pose) - drone.angular_impedance.theta_from_pose(updated_pose, obstacle_pose)
		pose_is_updated = True
		# obstacle-based feature set-point calculation
		center_ind = np.array([np.linalg.norm(x+y) for (x,y) in drone.traj[:,:2]-obstacle.position()[:2]]).argmin()#np.min( np.where(np.abs(drone.traj[:,:2]-obstacle.position()[:2])<0.01), axis=1 )[0]
		line = drone.traj[center_ind:,:2]
		circumference = obstacle.circle_points(R)
		X, Y = intersection(line, circumference)
		X = X + (obstacle.position()[0]-X)*0.2 # the feature set-point should be located inside the circumference
		Y = Y + (obstacle.position()[1]-Y)*0.2 # a bit close to the obstacle center
		drone.feature = np.array([X, Y, drone.sp[2]])
		# plt.figure()
		# plt.title(obstacle.name)
		# plt.plot(line[:,0], line[:,1])
		# plt.plot(circumference[:,0], circumference[:,1])
		# plt.plot(X,Y, 'ro')
		# plt.plot(drone.start[0], drone.start[1], '*')
		# # plt.axis('equal')
		# plt.show()
	else: 
		updated_pose = drone_pose
		drone.theta = None
		drone.d_theta[obstacle.id] = 0
		drone.delta = np.array([0,0])
		drone.near_obstacle[obstacle.id] = 0
		pose_is_updated = False
	if rad_imp:
		drone.delta = updated_pose - drone_pose
		if np.linalg.norm(drone.delta) > 0:
			updated_pose = drone.update_pose_radius(updated_pose, drone, obstacle.position()-drone.sp, koef=rad_imp_koef)
			# updated_pose -= rad_imp_koef*drone.delta
	updated_pose = np.append(updated_pose, drone.sp[2])

	return updated_pose, pose_is_updated

def force_impedance_model(R_layer, imp_pose_prev, imp_vel_prev, time_prev):
	F_coeff = 12 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	F = R_layer * F_coeff

	state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
	state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
	state_x = state_x[1]

	state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
	state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
	state_y = state_y[1]

	imp_pose = np.array( [state_x[0], state_y[0], 0] )
	imp_vel  = np.array( [state_x[1], state_y[1], 0] )
	return imp_pose, imp_vel, time_prev

def pose_update_force(drone):
	R_layers = np.array([1.0, 0.0])
	if sum(dists < R_layers[0]) > 0:
		drone.delta = updated_pose - drone_pose
		updated_pose = drone.update_pose_radius(updated_pose, drone, koef=5.0)



	# R_layers = np.array([1.0, 0.9, 0.7, 0.5, 0.3, 0.0])
	# dists = drone.dist_to_obstacles()
	# print (np.abs(R_layers - dists[0])).argmin()
	# print dists[0]

	# if sum(dists < R_layers[0]) > 0:
	# 	print "\nobstacles detected"
	# 	for l in range(len(R_layers)):
	# 		if sum(dists < R_layers[l]) > 0:
	# 			print "\nlayer", l

def quad_prog_circle(drone_pose, obstacle_pose, R):
	eq1 = np.array([ [obstacle_pose[0],1], [drone_pose[0],1] ])
	eq2 = np.array([obstacle_pose[1],drone_pose[1]])

	line_equation = np.linalg.solve(eq1, eq2)
	k = line_equation[0]
	b = line_equation[1]

	a_ = k**2+1
	b_ = 2*k*b  - 2*k*obstacle_pose[1] -2*obstacle_pose[0]
	c_ = obstacle_pose[1]**2 - R**2 + obstacle_pose[0]**2 - 2*b*obstacle_pose[1] + b**2

	D = (b_**2) - (4*a_*c_)
	if D>0:
		x_1 = (-b_-sqrt(D))/(2*a_)
		x_2 =  (-b_+sqrt(D))/(2*a_)

	y_1 = k * x_1 + b
	y_2 = k * x_2 + b

	point1 = np.array([ x_1, y_1])
	point2 = np.array([ x_2, y_2])

	dist_point1 = np.linalg.norm(point1 - drone_pose)
	dist_point2 = np.linalg.norm(point2 - drone_pose)

	if dist_point1 < dist_point2:
		updated_pose = point1
	else:
		updated_pose = point2

	return updated_pose


def recorder(cf1_name, cf2_name, cf3_name, human_name, obstacle_list, user_name, tacile_glove_on):
	if tacile_glove_on:
		user_name = user_name+'_with_glove'
	else:
		user_name = user_name+'_wo_glove'
	obstacle_topics = ''
	for i in range(len(obstacle_list)):
		obstacle_topics = obstacle_topics +" /vicon/"+obstacle_list[i]+"/"+obstacle_list[i]
	os.system("rosbag record -o /home/ruslan/Desktop/SwarmTouchData/"+user_name+" /vicon/"+cf1_name+"/"+cf1_name+" /vicon/"+cf2_name+"/"+cf2_name+" /vicon/"+cf3_name+"/"+cf3_name+" /vicon/"+human_name+"/"+human_name+obstacle_topics)
def start_recording(cf1_name, cf2_name, cf3_name, human_name, obstacle_list, user_name, tacile_glove_on):
	pose_recording = Process(target=recorder, args=(cf1_name, cf2_name, cf3_name, human_name, obstacle_list, user_name, tacile_glove_on,))
	pose_recording.start()
def killer_of_recorder():
	print 'killing the recorder'
	node_list = os.popen("rosnode list").read()
	print node_list
	for i in range(len(node_list)):
		if node_list[i:i+5] == '/reco':
			range_from = i
			range_to = i + 27
			break
	os.system('rosnode kill '+ node_list[range_from:range_to])
