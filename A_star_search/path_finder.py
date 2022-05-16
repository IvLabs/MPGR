#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from a_star import a_star_search
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import rospy
import numpy as np
from nav_msgs.msg import Path

    

def path_search(origin,destination,oc_grid):
    print(origin)
    print(oc_grid.shape)
    print(destination)

    route= a_star_search(origin,destination,oc_grid)
    path= Path()
    path.header.frame_id = "map"
    res= 0.030054
    #path= poses
    for i in route:
        row , col = i
        y = res*row - 50.01
        x = res*col - 50.01
        point= PoseStamped()
        point.pose.position.x , point.pose.position.y = x , y

        #add point to route
        path.poses.append(point)


    #rospy.init_node('Path',anonymous=True)
    
    path_topic= 'TrajectoryPlannerROS/global_plan'
    rate= rospy.Rate(10)    
    
    #creating a publisher for path
    path_pub= rospy.Publisher(path_topic, Path, queue_size=10)
    while not rospy.is_shutdown():
        #publishing the topic to rviz
        path_pub.publish(path)
        rate.sleep()
    


if __name__ == '__main__':
    try:
        print("yes")
        rospy.init_node('path_finder', anonymous=True)

        # obtaining grid
        grid=rospy.wait_for_message("/map",OccupancyGrid)
       # create an object to store map data
        grid_oc= grid.data

        #grid_oc is a 1D array. To convert it to 2D array we first convert
        #it to numpy array then reshape it
        oc_grid= np.array(grid_oc)
        oc_grid= np.reshape(oc_grid,(3328,3328))


        # obtaining start position 
        start=rospy.wait_for_message("/ground_truth/state",Odometry)

        x1 = start.pose.pose.position.x
        y1 = start.pose.pose.position.y

        print((x1,y1))

        res= 0.030054
        # converting real-time coordinates to occupancy grid indices 
        col1, row1= int((x1+50.01)/res) , int((y1+50.01)/res)
        origin = (row1, col1)


        
        # obtaining goal position
        goal=rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        x2 = goal.pose.position.x
        y2 = goal.pose.position.y
        res= 0.030054
        print((x2,y2))

        # converting real-time coordinates to occupancy grid indices 
        col2, row2= int((x2+50.01)/res) , int((y2+50.01)/res)
        destination = (row2,col2)

        
        
        
        path_search(origin, destination, oc_grid)
    
    except rospy.ROSInterruptException:
        pass
