#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf_name = "cf1"
    
    cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller

    cf.takeoff(targetHeight = 1.5, duration = 5.0)
    time.sleep(5.0)

    cf.goTo(goal = [0.1, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
    time.sleep(2.0)

    cf.land(targetHeight = 0.0, duration = 15.0)
    time.sleep(15.0)
    cf.stop()
