# -*- coding: utf-8 -*-
"""
Created on Thu Jan  6 17:48:30 2022

@author: karthik
"""

import imp
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

rospy.init_node('pathr', anonymous=True)
grid_oc= rospy.Subscriber('/map', OccupancyGrid, callback=None)

print(grid_oc)
rospy.spin()