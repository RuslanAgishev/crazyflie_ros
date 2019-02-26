#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

import swarmlib
import message_filters
from geometry_msgs.msg import PoseStamped, TransformStamped
import os
from multiprocessing import Process
import random
import numpy as np
from math import *

def start_recording():
    print "Data recording started"
    os.system("mkdir -p "+PATH+Subject_name)
    os.system("rosbag record -o "+PATH+Subject_name+"/vicon_data /vicon/cf1/cf1 /vicon/cf2/cf2 /vicon/cf3/cf3 /vicon/cf4/cf4")
    # os.system("rosbag record -o "+PATH+"poses /vicon/cf1/cf1 /vicon/cf2/cf2 /vicon/cf3/cf3 /vicon/cf4/cf4 /vicon/lp1/lp1 /vicon/lp2/lp2 /vicon/lp3/lp3 /vicon/lp4/lp4")

def land_detector():
    print "Land Detector..."
    land_time = - np.ones( min(len(drone_list), len(lp_list)) )
    switched_off = np.zeros(len(drone_list))
    while not rospy.is_shutdown():
        for drone in drone_list: drone.position()
        for lp in lp_list: lp.position()
        landed_drones_number = 0
        for d in range(len(drone_list)):
            for l in range(len(lp_list)):
                print abs(drone_list[d].pose[0] - lp_list[l].pose[0]), abs(drone_list[d].pose[1] - lp_list[l].pose[1]), abs(drone_list[d].pose[2] - lp_list[l].pose[2])
                if abs(drone_list[d].pose[0] - lp_list[l].pose[0])<0.10 and abs(drone_list[d].pose[1] - lp_list[l].pose[1])<0.10 and abs(drone_list[d].pose[2] - lp_list[l].pose[2])<0.07:
                    landed_drones_number += 1
                    # if land_time[i]==-1:
                    #     land_time[i] = time.time()-start_time
                    #     print("Drone %d is landed after %s seconds" % (i+1, land_time[i]))
                    if switched_off[d]==0:
                        print "Switch off the motors, %d drone" %d
                        for t in range(3): cf_list[d].stop()                  # stop motors
                    switched_off[d] = 1
                    # cf_list[i].setParam("tf/state", 0) # switch off LEDs
                    print landed_drones_number
                    if landed_drones_number==len(drone_list): rospy.signal_shutdown("landed")


rospy.init_node('test_high_level')

""" initialization """
# Names and variables
TAKEOFFHEIGHT = 1.8
data_recording = False
PATH = "~/Desktop/Swarm/Swarmskin/data/"


# cf_names = ['cf1', 'cf2', 'cf3', 'cf4']
cf_names = ['cf1', 'cf2', 'cf3', 'cf4']#, 'cf2', 'cf3']

lp_names = []
lp_names = ['lp1', 'lp2', 'lp3', 'lp4']
# lp_names = ['lp3', 'lp1', 'lp4']
drone_list = []
for name in cf_names:
    drone = swarmlib.Drone(name)
    drone_list.append(drone)
lp_list = []
for lp_name in lp_names:
    lp_list.append( swarmlib.Mocap_object(lp_name) )



# landing_velocity = random.choice([13,22]) #13,22
landing_velocity = 40
# landing_velocity = 22
# landing_velocity = 30
print "landing_velocity", landing_velocity



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

""" going to landing poses """
r = 0.3; theta1 = pi/4; theta2 = pi/4
l = 0.3 # distance between drones (arm length)
width = 0.45
human_pose = np.array([-0.8,0.0]); hx = human_pose[0]; hy = human_pose[1]
goto_arr = [ [hx+ (r+l)*cos(theta1), hy+ (r+l)*sin(theta1) +width/2],
             [hx+ r*cos(theta1),     hy+ r*sin(theta1) +width/2],
             [hx+ r*cos(theta2),     hy- r*sin(theta2) -width/2],
             [hx+ (r+l)*cos(theta2), hy- (r+l)*sin(theta2) -width/2] ]

for cf in cf_list:
    print "goto.. ", cf.prefix
    drone_id = int(cf.prefix[2])
    for t in range(3): cf.goTo(goal = goto_arr[drone_id-1]+[TAKEOFFHEIGHT], yaw=0.0, duration = 3.0, relative = False)
time.sleep(3.0)


""" landing """
print 'Landing...'
for cf in cf_list:
    for t in range(3): cf.land(targetHeight = -0.05, duration = landing_velocity)


# # if data_recording:
# #     print "Data recording started"
# #     pose_recording = Process(target=start_recording)
# #     pose_recording.start()


land_detector()







    




























    # cf.goTo(goal = [0.0, -0.4, 0.0], yaw=0.0, duration = 2.0, relative = True)
    # time.sleep(2.0)

    # cf.goTo(goal = [0.0, 0.8, 0.0], yaw=0.0, duration = 4.0, relative = True)
    # time.sleep(4.0)

    # cf.goTo(goal = [0.0, -0.4, 0.0], yaw=0.0, duration = 2.0, relative = True)
    # time.sleep(2.0)






    # time_to_sleep = 1.5
    # cf.goTo(goal = [.5, .5, 0.0], yaw=-.75*3.14, duration = 2.0, relative = True)
    # time.sleep(3.0)
    # for i in range(2):
    #     cf.goTo(goal = [0.0, -1.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    #     cf.goTo(goal = [-1.0, 0.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    #     cf.goTo(goal = [0.0, 1.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    #     cf.goTo(goal = [1.0, 0.0, 0.0], yaw=-1.57, duration = 2.0, relative = True)
    #     time.sleep(time_to_sleep)
    # cf.goTo(goal = [0, 0.0, 0.0], yaw=0, duration = 2.0, relative = False)
    # time.sleep(3.0)



    # traj1 = uav_trajectory.Trajectory()
    # traj1.loadcsv("takeoff.csv")

    # traj2 = uav_trajectory.Trajectory()
    # traj2.loadcsv("figure8.csv")

    # print(traj1.duration)

    # cf.uploadTrajectory(0, 0, traj1)
    # cf.uploadTrajectory(1, len(traj1.polynomials), traj2)

    # cf.startTrajectory(0, timescale=1.0)
    # time.sleep(traj1.duration * 2.0)

    # cf.startTrajectory(1, timescale=2.0)
    # time.sleep(traj2.duration * 2.0)

    # cf.startTrajectory(0, timescale=1.0, reverse=True)
    # time.sleep(traj1.duration * 1.0)

    # cf.stop()