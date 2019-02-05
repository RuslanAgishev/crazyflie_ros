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




def start_recording():
    print "Data recording started"
    os.system("mkdir "+Subject_name)
    os.system("rosbag record -o "+PATH+Subject_name+"/vicon_data /vicon/cf1/cf1 /vicon/cf2/cf2 /vicon/cf3/cf3 /vicon/cf4/cf4 /vicon/lp1/lp1 /vicon/lp2/lp2")
    # os.system("rosbag record -o "+PATH+"poses /vicon/cf1/cf1 /vicon/cf2/cf2 /vicon/cf3/cf3 /vicon/cf4/cf4 /vicon/lp1/lp1 /vicon/lp2/lp2 /vicon/lp3/lp3 /vicon/lp4/lp4")

def land_detector():
    land_time = - np.ones( min(len(drone_list), len(lp_list)) )
    while not rospy.is_shutdown():
        for drone in drone_list: drone.position()
        for lp in lp_list: lp.position()
        landed_drones_number = 0
        for i in range(min(len(drone_list), len(lp_list))):
            print abs(drone_list[i].pose[2] - lp_list[i].pose[2])
            if abs(drone_list[i].pose[0] - lp_list[i].pose[0])<0.07 and abs(drone_list[i].pose[1] - lp_list[i].pose[1])<0.07 and abs(drone_list[i].pose[2] - lp_list[i].pose[2])<0.1:
                landed_drones_number += 1
                if land_time[i]==-1:
                    land_time[i] = time.time()-start_time
                    print("Drone %d is landed after %s seconds" % (i+1, land_time[i]))
                if toFly:
                    cf_list[i].stop() # stop motors
                    cf_list[i].setParam("tf/state", 0) # switch off LEDs
                time.sleep(0.1)
        # TODO: stop data recording if all the drones landed
        # if data_recording and landed_drones_number>0:
        #     print 'kill the recorder'
        #     node_list = os.popen("rosnode list").read()
        #     os.system('rosnode kill '+node_list[72:99])
        #     print "Number of landed drones: " + str(landed_drones_number)
        #     if landed_drones_number==len(drone_list): rospy.signal_shutdown("landed")

def goto_land_pose(cf, TakeoffHeight, goalXY):
    print("Takeoff")
    cf.takeoff(targetHeight=TakeoffHeight, duration=5.0)
    time.sleep(5.0)

    cf.goTo(goal = [goalXY[0], goalXY[1], TakeoffHeight], yaw=0.0, duration=3.0, relative=False)
    time.sleep(3.0)

    print("Switch on LEDs")
    cf.setParam("tf/state", 4) # LED is ON
    time.sleep(1.0)
    global start_time
    start_time = time.time()

    print("Descending...")
    cf.land(targetHeight=-0.1, duration=landing_velocity)
    time.sleep(landing_velocity)


if __name__ == '__main__':
    rospy.init_node('swarmskin')

    """ initialization """
    # Names and variables
    TakeoffHeight  = 1.8
    data_recording = 0
    toFly          = 1
    lp_names = ['lp1', 'lp2']
    cf_names = ['cf1']
    # cf_names = ['cf1', 'cf2']
    # cf_names = ['cf1', 'cf2', 'cf3', 'cf4']
    PATH = "~/Desktop/Swarm/Swarmskin/data/"       

    drone_list = []
    for name in cf_names:
        drone = swarmlib.Drone(name)
        drone_list.append(drone)

    # landing pads init
    lp_list = []
    for lp_name in lp_names:
        lp_list.append( swarmlib.Mocap_object(lp_name) )

    # landing_velocity = random.choice([13,22]) #13,22
    landing_velocity = 13
    # landing_velocity = 22
    # landing_velocity = 30
    print "landing_velocity", landing_velocity


    # flight to landing positions
    # l=0.3; global_goal_poses = [ [ 0.0, l], [ l,   l], [ l,  -l], [ 0.0,-l] ]
    l=0.3; global_goal_poses = [ lp_list[0].position()[:2], lp_list[1].position()[:2], [ l,  -l], [ 0.0,-l] ]
    if toFly:
        # cfs init
        cf_list = []
        for cf_name in cf_names:
            cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
            cf.setParam("commander/enHighLevel", 1)
            cf.setParam("stabilizer/estimator", 2)  # Use EKF
            cf.setParam("stabilizer/controller", 2) # Use Mellinger controller
            time.sleep(0.1)
            cf_list.append(cf)
        threads = []
        for i in range(len(cf_names)):
            print cf_names[i]
            threads.append( Thread(target=goto_land_pose, args=(cf_list[i], TakeoffHeight, global_goal_poses[i],)) )
            threads[-1].start()


    if data_recording:
        try:
            Subject_name = sys.argv[1]
        except:
            print "\nEnter subject's name. For example:"
            print "rosrun crazyflie_demo swarmskin.py NAME"
            os.system("rosnode kill /swarmskin")
            sys.exit(1)
        pose_recording = Process(target=start_recording)
        pose_recording.start()



    # detect if drones touch landing pads
    land_detector()



