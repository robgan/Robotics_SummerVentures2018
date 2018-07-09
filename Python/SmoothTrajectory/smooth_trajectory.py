from __future__ import print_function
from math import atan2, cos, sin, sqrt, pi
import sys
import os
import numpy as np

cwd = os.getcwd()
sys.path.append('/'.join(cwd.split('/')[0:-1]))

from SummerVentures import SimpleVehicle, ang_diff, TrajectoryGenerator

def simple_trajectory():
	T=5
	dt=.1
	t=0
	        
	x = 0.0
	y = 0.0
	theta = 0.0
        
	x_obstacle = 10.0
	y_obstacle = 0.0

	x_waypoint = 10.0
	y_waypoint = 5.0

	x_goal = 20.0
	y_goal = 0.0

	vehicle = SimpleVehicle(x, y, theta, 1)

	traj = TrajectoryGenerator([0,0,0], [x_waypoint,y_waypoint,0], T, start_vel=[0,0,0],des_vel=[1,0,0])
	traj.solve()
	x_c = traj.x_c
	y_c = traj.y_c
	while x < x_obstacle:
                t= t + dt

                old_x = x
                old_y = y
                
                y= y_c[0,0] * t**5 + y_c[1,0] * t**4 + y_c[2,0] * t**3 + y_c[3,0]* t**2 + y_c[4,0]*t + y_c[5,0]
                x= x_c[0,0] * t**5 + x_c[1,0] * t**4 + x_c[2,0] * t**3 + x_c[3,0]* t**2 + x_c[4,0]*t + x_c[5,0]

                
                x_diff= x - old_x
                y_diff= y - old_y
                theta = atan2(y_diff,x_diff)


                vehicle.update_pose(x,y,theta) 
                vehicle.plot(goal=[x_goal,y_goal],xlims=[0,20],ylims=[-10,10], obstacles = [[x_obstacle,y_obstacle,150]])

      	traj = TrajectoryGenerator([x_waypoint,y_waypoint,0], [x_goal,y_goal,0], T, start_vel=[1,0,0],des_vel=[0,0,0])
	traj.solve()
	x_c = traj.x_c
	y_c = traj.y_c
        t=0.1
       	while x >=  x_obstacle:
                t= t + dt

                old_x = x
                old_y = y
                
                y= y_c[0,0] * t**5 + y_c[1,0] * t**4 + y_c[2,0] * t**3 + y_c[3,0]* t**2 + y_c[4,0]*t + y_c[5,0]
                x= x_c[0,0] * t**5 + x_c[1,0] * t**4 + x_c[2,0] * t**3 + x_c[3,0]* t**2 + x_c[4,0]*t + x_c[5,0]

                
                x_diff= x - old_x
                y_diff= y - old_y
                theta = atan2(y_diff,x_diff)


                vehicle.update_pose(x,y,theta) 
                vehicle.plot(goal=[x_goal,y_goal],xlims=[0,20],ylims=[-10,10], obstacles = [[x_obstacle,y_obstacle,150]])

if __name__ == '__main__':
        simple_trajectory()
