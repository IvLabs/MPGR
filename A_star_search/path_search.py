#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 14:19:17 2022

@author: dheeraj
"""
import rospy
from a_star import a_star_search
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import plot_path
import rospy
import numpy as np

# oc_grid = np.empty(1) 
# pos = []
# goal = []

def back_grid(msg):
    # create an object to store map data
    global oc_grid
    grid_oc= msg.data

    #grid_oc is a 1D array. To convert it to 2D array we first convert
    #it to numpy array then reshape it
    oc_grid= np.array(grid_oc)
    oc_grid.reshape(3328,3328)

    #return grid_oc



def start_callback(loc):
    global pos
    ox = loc.pose.pose.position.x
    oy = loc.pose.pose.position.y
    #oz = loc.pose.pose.position.z    
    pos = []
    pos.append(ox)
    pos.append(oy)
    # pos = Point()
    # # pos.x = loc.pose.pose.x
    # # pos.y = loc.pose.pose.y
    # # pos.z = loc.pose.pose.z
    # pos = loc.pose.pose.position
    # return pos

def goal_callback(target):
    global goal
    dx = target.pose.position.x
    dy = target.pose.position.y
    
    goal = []
    goal.append(dx)
    goal.append(dy)
    # goal = Point()
    # goal = target.pose.position
    # return goal

def path_search():
    
    # converting real-time coordinates to occupancy grid indices 
    x1,y1 = pos[0], pos[1]
    col1, row1= int((x1+50.01)/res) , int((y1+50.01)/res)
    origin = (row1,col1)
    x2 = goal[0] 
    y2 = goal[1]
    col2, row2= int((x2+50.01)/res) , int((y2+50.01)/res)
    destination = (row2,col2)
    
    while not rospy.is_shutdown():

        path= a_star_search(origin,destination,oc_grid)

        plot_path.plot(path)
        rospy.spin()

if __name__ == '__main__':
    try:
        res= 0.030054
        rospy.init_node('path_finder', anonymous=True)
        # obtaining start position 

        ori_sub = rospy.Subscriber('/ground_truth/state',Odometry, start_callback)
    
        # obtaining goal position
        
        destin_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_callback)   
        
        grid_sub= rospy.Subscriber('/map', OccupancyGrid, back_grid)
        rospy.Rate(1)

        
        

        

        path_search()
    
    except rospy.ROSInterruptException:
        pass

        