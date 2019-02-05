#!/usr/bin/env python

import argparse
import rospy
import tf_conversions
from crazyflie_driver.msg import FullState, Position
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
import geometry_msgs
import uav_trajectory
import pandas as pd
import numpy as np

def pos_callback(msg):
    global cf_pos
    cf_pos = msg


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("direction", type=str, help="stright or inverse trajectory")
    args = parser.parse_args()

    rospy.init_node('minsnap')

    direction = args.direction

    rate = rospy.Rate(40)

    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/world"

    cfname = "/crazyflie"
    pub = rospy.Publisher(cfname+"/cmd_position", Position, queue_size=1)
    stop_pub = rospy.Publisher(cfname+"/cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()
    pos_sub = rospy.Subscriber('/vicon/crazyflie18/crazyflie18', TransformStamped, pos_callback)

    start_time = rospy.Time.now()
    SCALE = 0.5
    traj = np.array( pd.read_csv('minjerk_trajectory.csv') ) * SCALE

    if direction=='inverse':
        RANGE = reversed(range(len(traj)))
    else:
        RANGE = range(len(traj))
    for i in RANGE:
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - start_time).to_sec()
        
        msg.x    = traj[i,0]
        msg.y    = traj[i,1]
        msg.z    = traj[i,2]
        msg.yaw = 0

        pub.publish(msg)
        print msg
        rate.sleep()

    # landing
    rospy.loginfo('Landing...')
    while not rospy.is_shutdown():
        msg.z -= 0.02
        if ( cf_pos.transform.translation.z < 0.11 ):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    stop_pub.publish(stop_msg)
