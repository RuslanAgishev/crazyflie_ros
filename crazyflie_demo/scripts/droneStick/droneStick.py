#!/usr/bin/env python

import numpy as np
import time
import swarmlib
from swarmlib import Drone
import rospy
import math

import crazyflie
from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position

import matplotlib.pyplot as plt
from potential_fields import *
from tools import *

def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
    name = cf_name + "/cmd_position"
    msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
    pub = rospy.Publisher(name, Position, queue_size=1)
    pub.publish(msg)

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


class Robot(Drone):
    def __init__(self, name):
        Drone.__init__(self, name)
        # self.route = np.array([self.sp])
        self.f = 0
        self.leader = False

    def local_planner(self, obstacles_poses, params):
        self.f = combined_potential(obstacles_poses, params.R_obstacles, self.sp[:2], params.obstacles_influence_radius)
        self.sp[:2] = gradient_planner(self.sp[:2], self.f)
        # self.route = np.vstack( [self.route, self.sp] )

class Params:
    def __init__(self,):
        self.R_obstacles = 0.15 # [m]
        self.obstacles_influence_radius = 1.4
        self.l = 0.4 # [m], inter-drones distance

rospy.init_node('CrazyflieAPI', anonymous=False)

toFly          = 0
TAKEOFFHEIGHT  = 1.4
TAKEOFFTIME    = 5.0
LANDTIME       = 2.0
initialized    = False
vel_koef       = 3.0
put_limits       = 1
limits           = np.array([ 2.0, 2.0, 2.5 ]) # limits desining safety flight area in the room
limits_negative  = np.array([ -2.0, -2.0, -0.1 ])
repel_robots   = 1

params = Params()

# joystick
drone_joystick = Drone("cf4")

# drones-followers
cf_names = ['cf1', 'cf2', 'cf3']
# cf_names = ['cf2']


drones = []
drones_poses = []
for name in cf_names:
    drone = Robot(name)
    drones.append( drone )
    drones_poses.append(drone.position())

obstacles = []
obstacles_poses = []
obstacles_names = ['obstacle4', 'obstacle10', 'obstacle12', 'obstacle25']
for name in obstacles_names:
    obstacle = swarmlib.Obstacle(name)
    obstacles.append( obstacle )
    obstacles_poses.append(obstacle.position()[:2])
    

if __name__ == "__main__":
    if toFly:
        cf_list = []
        for cf_name in cf_names:
            # print "adding.. ", cf_name
            cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
            cf.setParam("commander/enHighLevel", 1)
            cf.setParam("stabilizer/estimator",  2) # Use EKF
            cf.setParam("stabilizer/controller", 2) # Use mellinger controller
            cf_list.append(cf)
        for t in range(3):
            # print "takeoff.. ", cf.prefix
            for cf in cf_list:
                cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = TAKEOFFTIME)
        time.sleep(TAKEOFFTIME)
        # for t in range(3):
        #     for cf in cf_list: cf.land(targetHeight = -0.05, duration = 3.0)


    print 'start DroneStick \n'
    # time_to_play = 300
    # for i in range(time_to_play*100):
    plt.figure(figsize=(8,8))
    while not rospy.is_shutdown():
        drone_orient = drone_joystick.orientation()
        drone_joystick.position()
        for drone in drones: drone.pose = drone.position()
        roll = drone_joystick.orient[0]
        pitch = drone_joystick.orient[1]

        # update obstacles poses
        for i in range(len(obstacles)):
            obstacles_poses[i] = obstacles[i].position()[:2]

        # update drones poses
        for i in range(len(drones)):
            drones_poses[i] = drones[i].sp[:2]

        # drones formation:
        # drones[1].sp = drones[0].sp + np.array([-0.86*params.l , params.l/2., 0])
        # drones[2].sp = drones[0].sp + np.array([-0.86*params.l ,-params.l/2., 0])

        if not initialized:
            for drone in drones: drone.sp = np.array( [drone.pose[0], drone.pose[1], TAKEOFFHEIGHT] )
            time_prev = time.time()
            initialized = True

        pitch_thresh = 0.05
        roll_thresh = 0.05
        if abs(pitch)<pitch_thresh:
            x_input = 0
        else:
            x_input = pitch
        if abs(roll)<roll_thresh:
            y_input = 0
        else:
            y_input = -roll

        # z_disp = drone.pose[2] - set_point[2]
        # if abs(z_disp)<0.015:
        #     z_input = 0
        # else:
        #     z_input = z_disp

        cmd_vel = vel_koef*(np.array([x_input, y_input, 0]))
        # print 'cmd_vel', cmd_vel
        # np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.035), 0)
        # np.putmask(cmd_vel, abs(cmd_vel) <= (0.20), 0)
        time_now = time.time()
        for drone in drones: drone.sp += cmd_vel*(time.time()-time_prev)
        time_prev = time_now

        if put_limits:
            for drone in drones:
                np.putmask(drone.sp, drone.sp >= limits, limits)
                np.putmask(drone.sp, drone.sp <= limits_negative, limits_negative)

        # correct point to follow with local planner
        for p in range(len(drones)):
            if repel_robots:
                robots_obstacles = [x for i,x in enumerate(drones_poses) if i!=p] # all poses except the robot[p]
                obstacles_poses1 = obstacles_poses + robots_obstacles
                drones[p].local_planner(obstacles_poses1, params)
            else:
                drones[p].local_planner(obstacles_poses, params)

        

        if toFly:
            for drone in drones: drone.fly()

        # visualization: RVIZ
        for drone in drones:
            drone.publish_sp()
            drone.publish_path_sp()
        for obstacle in obstacles: obstacle.publish_position()
        # visualization: matplotlib
        plt.cla()
        draw_gradient(drones[0].f)
        draw_map(obstacles_poses, params.R_obstacles)
        for drone in drones: plt.plot(drone.sp[0], drone.sp[1], '^', markersize=10, label=drone.name)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.draw()
        plt.pause(0.01)
        # time.sleep(0.01)



        if (drones[0].pose[2] - drone_joystick.pose[2])>0.7:
            print "Finish"

            print 'Landing!!!'
            for drone in drones: drone.sp = drone.position()
            print "drone_follower1_landing_pose", drones[0].sp
            while(1):
                for drone in drones:
                    drone.sp[2] = drone.sp[2]-0.02
                    drone.fly()

                if drones[0].sp[2]<-1.0:
                    print 'reached the floor'
                    break
                time.sleep(0.1)
                if toFly:
                    for cf in cf_list: cf.stop()

            break

