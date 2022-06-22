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

from rdp import rdp


def rdp_path(path):
    points = np.array(path)
    new_pts = rdp(points, epsilon= 0.7)
    print(len(new_pts))
    # print(new_pts)
    #nx ,ny =np.split(new_pts,2,axis=1)
    return new_pts
    

def path_search(origin,destination,oc_grid):
    print('start', origin)
    # print(oc_grid.shape)
    print('goal',destination)

    route= a_star_search(origin,destination,oc_grid)

    new_route = rdp_path(route)
    path= Path()
    path_rdp = Path()

    path.header.frame_id = "map"
    path_rdp.header.frame_id = "map"

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

    for i in new_route:
        row , col = i
        y = res*row - 50.01
        x = res*col - 50.01
        point= PoseStamped()
        point.pose.position.x , point.pose.position.y = x , y

        #add point to route
        path_rdp.poses.append(point)


    #rospy.init_node('Path',anonymous=True)
    
    print("Og path", len(route), 'points')
    print("new path", len(new_route), 'points')
    print(new_route)


    path_topic= '/global_plan'
    path_topic_rdp= '/global_plan/rdp'
    rate= rospy.Rate(5)    
    
    #creating a publisher for path
    path_pub= rospy.Publisher(path_topic, Path, queue_size=10)
    path_pub_rdp = rospy.Publisher(path_topic_rdp, Path, queue_size=10)
    while not rospy.is_shutdown():
        #publishing the topic to rviz
        path_pub.publish(path)
        path_pub_rdp.publish(path_rdp)
        rate.sleep()
    


if __name__ == '__main__':
    try:
        
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

        res= 0.030054
        # converting real-time coordinates to occupancy grid indices 
        col1, row1= int((x1+50.01)/res) , int((y1+50.01)/res)
        origin = (row1, col1)


        
        # obtaining goal position
        goal=rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        x2 = goal.pose.position.x
        y2 = goal.pose.position.y
        res= 0.030054
        # converting real-time coordinates to occupancy grid indices 
        col2, row2= int((x2+50.01)/res) , int((y2+50.01)/res)
        destination = (row2,col2)

        
        
        
        path_search(origin, destination, oc_grid)
    
    except rospy.ROSInterruptException:
        pass
