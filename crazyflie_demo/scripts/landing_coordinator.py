#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

import swarmlib
import message_filters
from geometry_msgs.msg import PoseStamped, TransformStamped
import os
import sys
from multiprocessing import Process
from threading import Thread
import random
import numpy as np
from numpy.linalg import norm
from math import *




def start_recording():
    print "Data recording started"
    os.system("mkdir -p "+PATH+Subject_name)
    os.system("rosbag record -o "+PATH+Subject_name+"/vicon_data /vicon/cf1/cf1 /vicon/cf2/cf2 /vicon/cf3/cf3 /vicon/cf4/cf4")
    # os.system("rosbag record -o "+PATH+"poses /vicon/cf1/cf1 /vicon/cf2/cf2 /vicon/cf3/cf3 /vicon/cf4/cf4 /vicon/lp1/lp1 /vicon/lp2/lp2 /vicon/lp3/lp3 /vicon/lp4/lp4")

def land_detector():
    for lp in lp_list: lp.pose = lp.position()
    # print abs(drone_list[0].pose[0] - lp_list[0].pose[0]), abs(drone_list[0].pose[1] - lp_list[0].pose[1]), abs(drone_list[0].pose[2] - lp_list[0].pose[2])
    landed_drones_number = 0
    for i in range(min(len(drone_list), len(lp_list))):
        if abs(drone_list[i].pose[0] - lp_list[i].pose[0])<0.07 and abs(drone_list[i].pose[1] - lp_list[i].pose[1])<0.07 and abs(drone_list[i].pose[2] - lp_list[i].pose[2])<0.05:
        # if abs(drones[i].sp[0] - lp_list[i].pose[0])<0.07 and abs(drones[i].sp[1] - lp_list[i].pose[1])<0.07 and abs(drones[i].sp[2] - lp_list[i].pose[2])<0.07:
            landed_drones_number += 1
            if toFly:
                print "Stop motors %d drone" %i
                for t in range(5): cf_list[i].stop()
            if landed_drones_number==len(drone_list): rospy.signal_shutdown("landed")


def flight(cf_list, TakeoffHeight, goal_poses_XY):
    num_commands = 5
    for t in range(num_commands):
        print("Takeoff")
        for cf in cf_list:
            cf.takeoff(targetHeight=TakeoffHeight, duration=6.0)
    time.sleep(6.0)



def hover():
    print "hovering..."
    if toFly:
        print "Switch on LEDs"
        for cf in cf_list: cf.setParam("tf/state", 4) # LED is ON
    while not rospy.is_shutdown():
        for i in range(200):
            for drone in drone_list:
                if toFly: drone.fly()
                drone.publish_sp()
                drone.publish_path()
                rate.sleep()
        break



if __name__ == '__main__':
    rospy.init_node('swarmskin')

    """ initialization """
    TakeoffHeight  = 2.0
    data_recording = 0
    toFly          = 0
    TimeFlightSec    = 3 # [s]
    ViconRate        = 200 # [Hz]
    N_samples        = ViconRate * TimeFlightSec
    lp_names = []
    # lp_names = ['lp2', 'lp4']
    cf_names = ['cf1', 'cf2', 'cf3', 'cf4']

    PATH = "~/Desktop/Swarm/Swarmskin/data/"
    area_center = np.array([0,0]) # approximate human location
    radial_poses = [0, pi/2, pi, 3*pi/2]
    R = 1.0; global_goal_poses = np.array([ [R,0], [0,R], [-R,0], [0,-R] ]) + area_center

    # landing pads init
    lp_list = []
    for lp_name in lp_names:
        lp_list.append( swarmlib.Mocap_object(lp_name) )


    if toFly:
        cf_list = []
        for cf_name in cf_names:
            cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
            cf.setParam("commander/enHighLevel", 1)
            cf.setParam("stabilizer/estimator",  2) # Use EKF
            cf.setParam("stabilizer/controller", 2) # Use Mellinger controller
            time.sleep(0.1)
            cf_list.append(cf)

        flight(cf_list, TakeoffHeight, global_goal_poses)


    drone_list = []
    for name in cf_names:
        drone = swarmlib.Drone(name)
        drone_list.append(drone)
        drone_list[-1].sp = drone_list[-1].position()

    # """ trajectory generation """
    start_poses = []
    for drone in drone_list: start_poses.append( drone.sp )
    for i in range(len(drone_list)):
        drone_list[i].traj = np.array([np.linspace(start_poses[i][0], global_goal_poses[i][0], N_samples),
                                       np.linspace(start_poses[i][1], global_goal_poses[i][1], N_samples),
                                       np.linspace(start_poses[i][2], TakeoffHeight,    N_samples)]).T

    sp_ind = 0
    print "Going to human location..."
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        if sp_ind >= N_samples-1: sp_ind = N_samples-1 # holding the last pose
        for drone in drone_list: drone.sp = drone.traj[sp_ind,:]
        sp_ind += 1

        # TO FLY
        if toFly:
            for drone in drone_list: drone.fly()

        # TO VISUALIZE
        for drone in drone_list:
            drone.publish_sp()
            drone.publish_path(limit=N_samples)

        """ Searching landing pads """
        if sp_ind == N_samples-1:
            # hover and LEDs switch ON
            hover()
            print 'Searching landing pads...'
            rate_land = rospy.Rate(20)
            N = 100; T = np.linspace(0,2*pi,N); t = 0
            r = R
            while not rospy.is_shutdown():
                for i in range(len(drone_list)):
                    drone_list[i].position()
                    drone_list[i].sp[0] = area_center[0] + r * cos(T[t]+radial_poses[i])
                    drone_list[i].sp[1] = area_center[1] + r * sin(T[t]+radial_poses[i])
                    r -= 0.0002
                    if r < R/2: r = R/2
                if toFly:
                    for drone in drone_list: drone.fly()
                for drone in drone_list:
                    drone.publish_sp()
                    drone.publish_path_sp()
                t+=1; t = t%N

                rate_land.sleep()

        rate.sleep()

    print "Done"
    





