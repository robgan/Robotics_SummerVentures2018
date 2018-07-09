from __future__ import print_function
from math import atan2, cos, sin, sqrt, pi
import sys
import os
import numpy as np

cwd = os.getcwd()
sys.path.append('/'.join(cwd.split('/')[0:-1]))

from SummerVentures import SimpleVehicle, ang_diff

def simple_trajectory():
	Kv = 3
	Kh = 6
	Ki = 0.00001
	d = 2
	dt = 0.1
        dx = .1
        
	x = 0.0
	y = 0.0
	theta = 0.0
        t= 0
        
	x_obstacle = 10.0
	y_obstacle = 0.0

	x_waypoint = 10.0
	y_waypoint = 5.0

	x_goal = 20.0
	y_goal = 0.0

	m1 = (y_waypoint-y)/(x_waypoint-x)
	b1 = y_waypoint-m1*x_waypoint

	m2 = (y_goal-y_waypoint)/(x_goal-x_waypoint)
	b2 = y_goal-m2*x_goal

	x_pursuit = x
	y_pursuit = y

	vehicle = SimpleVehicle(x, y, theta, 1)

	x_diff = x_pursuit-x
	y_diff = y_pursuit-y
	
	error = sqrt(x_diff**2 + y_diff**2)-d
	totalError = 0
	T = 10000

        A_x = np.array([[0,0,1], [(T**2)/4,T/2,1], [T**2,T,1]])
        b_x = np.array([[0],[x_waypoint],[x_goal]])
        x_coeff = np.linalg.solve(A_x,b_x)
        
        A_y = np.array([[0,0,1],[(T**2)/4,T/2,1],[T**2,T,1]])
        b_y = np.array([[0],[y_waypoint],[y_goal]])
        y_coeff = np.linalg.solve(A_y,b_y)

        
        
	while sqrt((x-x_goal)**2+(y-y_goal**2)**2) > 1:
		#############YOUR CODE GOES HERE#############
                t= t + dt

                old_x = x
                old_y = y
                
                y= y_coeff[0.0] * t**2 + y_coeff[1,0] * t + x_coeff[2,0]
                x = x_coeff[0,0] * t**2 + x_coeff[1,0] * t + x_coeff[2,0]

                
                x_diff= x - old_x
                y_diff= y - old_y
                theta = atan2(y_diff,x_diff)


                vehicle.update_pose(x,y,theta) 
                vehicle.plot(goal=[x_goal,y_goal],xlims=[0,20],ylims=[-10,10], obstacles = [[x_obstacle,y_obstacle,150]])
	        #############################################

if __name__ == '__main__':
        simple_trajectory()
