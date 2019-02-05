#!/usr/bin/env python

import rospy
import tf
import crazyflie
import time
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
import numpy as np

def trajgen(crazyflie, mode='setpoint_relative',number_of_points=200, x_des=0,y_des=0,z_des=0):
    x0 = crazyflie.transform.translation.x
    y0 = crazyflie.transform.translation.y
    z0 = crazyflie.transform.translation.z
    if mode=='setpoint_global':
        x = np.linspace(x0,x_des,number_of_points)
        y = np.linspace(y0,y_des,number_of_points)
        z = np.linspace(z0,z_des,number_of_points)
    elif mode=='setpoint_relative':
        x = np.linspace(x0,x0+x_des,number_of_points)
        y = np.linspace(y0,y0+y_des,number_of_points)
        z = np.linspace(z0,z0+z_des,number_of_points)
    return x,y,z


def joy_cb(data):
    global joy
    joy = data


def cf18_cb(data):
    global crazyflie18
    crazyflie18 = data

def cf15_cb(data):
    global crazyflie15
    crazyflie15 = data

if __name__ == '__main__':
    rospy.init_node('binded2', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    cf18 = crazyflie.Crazyflie("crazyflie18", "/vicon/crazyflie18/crazyflie18")
    cf15 = crazyflie.Crazyflie("crazyflie15", "/vicon/crazyflie15/crazyflie15")

    rate = rospy.Rate(10) # 10 hz

    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = 0.0
    msg.y = 0.0
    msg.z = 0.0
    msg.yaw = 0.0

    target_pub18 = rospy.Publisher("/crazyflie18/cmd_position", Position, queue_size=1)
    target_pub15 = rospy.Publisher("/crazyflie15/cmd_position", Position, queue_size=1)
    cf18_sub = rospy.Subscriber("/vicon/crazyflie18/crazyflie18", TransformStamped, cf18_cb)
    cf15_sub = rospy.Subscriber("/vicon/crazyflie15/crazyflie15", TransformStamped, cf15_cb)
    joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)

    stop_pub18 = rospy.Publisher("/crazyflie18/cmd_stop", Empty, queue_size=1)
    stop_pub15 = rospy.Publisher("/crazyflie15/cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

    time.sleep(2)
    #HOME-locations:
    x_home18 = crazyflie18.transform.translation.x
    y_home18 = crazyflie18.transform.translation.y
    x_home15 = crazyflie15.transform.translation.x
    y_home15 = crazyflie15.transform.translation.y

    rospy.loginfo('Home locations 18: '+str(x_home18)+', '+str(y_home18))
    rospy.loginfo('Home locations 15: '+str(x_home15)+', '+str(y_home15))
    rospy.loginfo("Press green A-button for takeoff")
    while not rospy.is_shutdown():
        try:
            if joy.buttons[0]==1:
                break
        except:
            pass
        rate.sleep()


    # LOW_LEVEL-takeoff
    HEIGHT = 1.0
    rospy.loginfo('Takeoff at '+str(HEIGHT))
    while not rospy.is_shutdown():
        N = 25
        for z in range(N):
            msg.x = x_home18
            msg.y = y_home18
            msg.yaw = 0.0
            msg.z = HEIGHT / N
            now = rospy.get_time()
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            target_pub18.publish(msg)
            msg.x = x_home15
            msg.y = y_home15
            msg.header.stamp = rospy.Time.now()
            target_pub15.publish(msg)
            rate.sleep()
        for z in range(N):
            msg.x = x_home18
            msg.y = y_home18
            msg.yaw = 0.0
            msg.z = HEIGHT
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            target_pub18.publish(msg)
            msg.x = x_home15
            msg.y = y_home15
            msg.header.stamp = rospy.Time.now()
            target_pub15.publish(msg)
            rate.sleep()
        break

    rospy.loginfo("Press green B-button for landing")
    while not rospy.is_shutdown():
        msg.x = x_home15
        msg.y = y_home15
        msg.header.stamp = rospy.Time.now()
        target_pub15.publish(msg)
        msg.x = x_home18
        msg.y = y_home18
        msg.header.stamp = rospy.Time.now()
        target_pub18.publish(msg)
        try:
            if joy.buttons[1]==1:
                break
        except:
            pass
        rate.sleep()

    # LOW_LEVEL-land
    rospy.loginfo('Landing...')
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.x = x_home18
        msg.y = y_home18
        msg.z -= 0.02
        #msg.yaw = 0.0
        now = rospy.get_time()
        if ( msg.z < -0.02 ):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        target_pub18.publish(msg)
        msg.x = x_home15
        msg.y = y_home15
        msg.header.stamp = rospy.Time.now()
        target_pub15.publish(msg)
        rate.sleep()

    rospy.loginfo('End')
    stop_pub18.publish(stop_msg)
    stop_pub15.publish(stop_msg)
