#!/usr/bin/env python

"""
In order to launch the script execute:

roscore
python3 GradientBasedPlanner.py
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.morphology import distance_transform_edt as bwdist
import xlwt
import time

""" ROS """
import rospy
from geometry_msgs.msg import TransformStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
import crazyflie



def GradientBasedPlanner (f, start_coords, end_coords, max_its):
    """
    GradientBasedPlanner : This function plans a path through a 2D
    environment from a start to a destination based on the gradient of the
    function f which is passed in as a 2D array. The two arguments
    start_coords and end_coords denote the coordinates of the start and end
    positions respectively in the array while max_its indicates an upper
    bound on the number of iterations that the system can use before giving
    up.
    The output, route, is an array with 2 columns and n rows where the rows
    correspond to the coordinates of the robot as it moves along the route.
    The first column corresponds to the x coordinate and the second to the y coordinate
	"""
    [gy, gx] = np.gradient(-f);

    route = np.vstack( [np.array(start_coords), np.array(start_coords)] )
    for i in range(max_its):
        current_point = route[-1,:];
        to_goal = sum( abs(current_point - end_coords) )
        # print(to_goal)
        if to_goal < 10.0: # cm
            print('Reached the goal !')
            print('Distance to goal [cm]: '+str(round(to_goal,2)))
            route = route[1:,:]
            return route
        ix = int(round( current_point[1] ));
        iy = int(round( current_point[0] ));
        # w = 2 # w=2 - min smoothing window to estimate mean gradient
        w = 20
        vx = np.mean(gx[ix-int(w/2):ix+int(w/2), iy-int(w/2):iy+int(w/2)])
        vy = np.mean(gy[ix-int(w/2):ix+int(w/2), iy-int(w/2):iy+int(w/2)])
        dt = 1 / np.linalg.norm([vx, vy]);
        next_point = current_point + dt*np.array( [vx, vy] );
        route = np.vstack( [route, next_point] );
    route = route[1:,:]
    print('The goal is not reached after '+str(max_its)+' iterations...')
    print('Distance to goal [cm]: '+str(round(to_goal,2)))
    return route

def CombinedPotential(obstacle, goal):
	""" Repulsive potential """
	d = bwdist(obstacle==0);
	d2 = (d/100.) + 1; # Rescale and transform distances
	d0 = 2;
	nu = 500; # repulsive strength: 500 is stronger than 100
	repulsive = nu*((1./d2 - 1/d0)**2);
	repulsive [d2 > d0] = 0;

	""" Attractive potential """
	xi = 1/700. # attractive strength: 1/100 is stronger than 1/500
	attractive = xi * ( (x - goal[0])**2 + (y - goal[1])**2 );

	""" Combine terms """
	f = attractive + repulsive;
	return f


def meters2grid(pose_m, nrows=500, ncols=500):
	# [0, 0](m) -> [250, 250]
	# [1, 0](m) -> [250+100, 250]
	# [0,-1](m) -> [250, 250-100]
	pose_on_grid = np.array(pose_m)*100 + np.array([ncols/2, nrows/2])
	return np.array( pose_on_grid, dtype=int)

def grid2meters(pose_grid, nrows=500, ncols=500):
	# [250, 250] -> [0, 0](m)
	# [250+100, 250] -> [1, 0](m)
	# [250, 250-100] -> [0,-1](m)
	pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100
	return pose_meters

def savedata(data, figure, path):
    #style0 = xlwt.easyxf('font: name Times New Roman, color-index red, bold on', num_format_str='#,##0.00')
    #style1 = xlwt.easyxf(num_format_str='D-MMM-YY')
    wb = xlwt.Workbook()
    ws = wb.add_sheet('route')
    ws.write(0, 0, 'X, [m]'); ws.write(0, 1, 'Y, [m]')
    for row in range(1,route.shape[0]):
        ws.write(row, 0, route[row, 0]); ws.write(row, 1, route[row, 1])
    wb.save(path+'output.xls')
    figure.savefig(path+'route.png', dpi=fig.dpi)    


""" initialization """
visualize     = 1
toFly         = 0
direction     = 'forward'
TakeoffTime   = 3.0 # [s]
TakeoffHeight = 0.5 # [m]
goal          = meters2grid([1.0,-1.0]);
# goal        = meters2grid([-1.0,1.0]);
R_drone       = 0.1 # [m]
R_obstacles   = 0.05; R_obstacles += R_drone # [m]

cf_names         = np.array(['cf1',
                             'cf2',
                             'cf3'
                             ])



""" Generate obstacles map """
nrows = 500; ncols = 500; # room 5x5 meters
obstacle = np.zeros((nrows, ncols));
[x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))


# rectangular obstacles
# obstacle [300:, 100:250] = True;
# obstacle [150:200, 400:500] = True;

def pos_callback(data):
    global cf_pose
    cf_pose = [data.transform.translation.x, data.transform.translation.y, data.transform.translation.z]

def callback0(data):
    global pose0
    pose0 = [data.transform.translation.x, data.transform.translation.y]

def callback3(data):
    global pose3
    pose3 = [data.transform.translation.x, data.transform.translation.y]

def callback4(data):
    global pose4
    pose4 = [data.transform.translation.x, data.transform.translation.y]

rospy.init_node('motion_planning', anonymous=True)
pos_sub = rospy.Subscriber('/vicon/'+cf_names[2]+'/'+cf_names[2], TransformStamped, pos_callback)

rospy.Subscriber("/vicon/obstacle0/obstacle0", TransformStamped, callback0)
rospy.Subscriber("/vicon/obstacle3/obstacle3", TransformStamped, callback3)
rospy.Subscriber("/vicon/obstacle4/obstacle4", TransformStamped, callback4)


time.sleep(1) # wait for ROS-Vicon driver to start recieving data
try:
    real_obstacles = [pose0, pose3, pose4]
except:
    real_obstacles = []
fake_obstacles = [[-2, 1], [1.5, 0.5], [0, 0], [-1.8, -1.8]]
obstacles_poses = real_obstacles + fake_obstacles # 2D - coordinates [m]
# obstacles_poses = [pose0, pose3, pose4] # 2D - coordinates [m]

for pose in obstacles_poses:
	pose = meters2grid(pose)
	x0 = pose[0]; y0 = pose[1]
	# cylindrical obstacles
	t = ((x - x0)**2 + (y - y0)**2) < (100*R_obstacles)**2
	obstacle[t] = True;


""" Plan route """
# start = meters2grid(cf_pose[:2]);
start = meters2grid([-2.0, 2.0]);


f = CombinedPotential(obstacle, goal)
route = GradientBasedPlanner(f, start, goal, max_its=1000);
route = grid2meters(route)

""" Visualization """
[gy, gx] = np.gradient(-f);

# Velocities map and a Path from start to goal
skip = 10;
[x_m, y_m] = np.meshgrid(np.linspace(-2.5, 2.5, ncols), np.linspace(-2.5, 2.5, nrows))
start_m = grid2meters(start); goal_m = grid2meters(goal)
fig = plt.figure(figsize=(nrows/50, ncols/50))
ax = plt.gca()
Q = plt.quiver(x_m[::skip, ::skip], y_m[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip])
plt.plot(start_m[0], start_m[1], 'ro', markersize=5);
plt.plot(goal_m[0], goal_m[1], 'ro', color='green', markersize=5);
plt.plot(route[:,0], route[:,1], linewidth=3);
plt.xlabel('X')
plt.ylabel('Y')
# obstacles
for pose in real_obstacles:
    circle = plt.Circle(pose, R_obstacles, color='yellow')
    ax.add_artist(circle)
for pose in fake_obstacles:
    circle = plt.Circle(pose, R_obstacles, color='blue')
    ax.add_artist(circle)

PATH = '/home/ruslan/Desktop/'
# savedata(route, fig, PATH)
if visualize:
    plt.show()

""" Flight: takeoff -> trajectory following -> landing """
if toFly:
    print("takeoff")
    cf1 = crazyflie.Crazyflie(cf_names[2], '/vicon/'+cf_names[2]+'/'+cf_names[2])
    cf1.setParam("commander/enHighLevel", 1)
    cf1.setParam("stabilizer/estimator", 2) # Use EKF
    cf1.setParam("stabilizer/controller", 2) # Use Mellinger controller
    cf1.takeoff(targetHeight = TakeoffHeight, duration = TakeoffTime)
    time.sleep(TakeoffTime+1)

    cf1.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(3.0)

    # rate = rospy.Rate(40)

    # msg = Position()
    # msg.header.seq = 0
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = "/world"

    # cfname = cf_names[0]
    # pub = rospy.Publisher(cfname+"/cmd_position", Position, queue_size=1)
    # stop_pub = rospy.Publisher(cfname+"/cmd_stop", Empty, queue_size=1)
    # stop_msg = Empty()

    # print('Executing trajectory...')
    # start_time = rospy.Time.now()
    # for i in range(len(route)):
    #     msg.header.seq += 1
    #     msg.header.stamp = rospy.Time.now()
    #     t = (msg.header.stamp - start_time).to_sec()
        
    #     msg.x    = route[i,0]
    #     msg.y    = route[i,1]
    #     msg.z    = TakeoffHeight
    #     msg.yaw  = 0

    #     pub.publish(msg)
    #     rate.sleep()
    # print('Trajectory time: ', str(round(t,2))+' sec')

    # print('Landing...')
    # while not rospy.is_shutdown():
    #     msg.z -= 0.02
    #     if ( msg.z < -1.0 ):
    #         time.sleep(1)
    #         # cf1.stop()
    #         print('reached the floor, shutdown')
    #         rospy.signal_shutdown('landed')
    #     msg.header.seq += 1
    #     msg.header.stamp = rospy.Time.now()
    #     pub.publish(msg)
    #     rate.sleep()
    # stop_pub.publish(stop_msg)

