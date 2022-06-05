#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
from std_srvs.srv import Empty
import message
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

initial = Odometry()
goal = PoseStamped()


p = 50.010000
r =  0.030000

# neighbourhood nodes
def neighbour(v_node, img34):
        # next_node list to store accessed neighbours from current node u
    next_node = []
    row, col = v_node
    for i in range(-1,2):
        for j in range(-1,2):
            if i == 0 and  j ==0:
                continue
            distance = float(((i**2+j**2)**0.5))
            pixel = img34[row+i, col+j]
            #print(pixel)
            if pixel == 0:
                next_node.append((row+i, col+j,distance))
            elif pixel == 100:
                distance = 10000000000
                next_node.append((row+i, col+j,distance))
            elif pixel == -1:
                distance = 10000000000
                next_node.append((row+i, col+j,distance))
                
                
    return next_node
    

def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
    return path


def distance_heuristic(state_key, goal_k):
    a,b = state_key
    c,d = goal_k
    d = float(((c-a)**2 + (d-b)**2 )**0.5)
    return d

def a_star_search(origin_key, goal_key, img):
    open_queue = message.priority_dict({})
    # to store processed vertices
    closed_dict = {}
    # dictionary of predecessors for each vertex
    predecessors = {}
    
    predecessors[origin_key]= 0
    costs = {}
    costs[origin_key] = 0.0
    open_queue[origin_key] = distance_heuristic(origin_key,goal_key)

   
    goal_found = False
    while (open_queue):
        # print(len(predecessors))
        u,l= open_queue.pop_smallest()
        # print(u)
        ucost = costs[u]
        
        # by popping if we get required goal
        if u == goal_key:
            goal_found = True
            break
        
        # For outgoing edges of current vertex
        for dist in neighbour(u,img):
            v = (dist[0],dist[1])
            # to get distance to the next vertex
            uv_cost = float(dist[2]) 
            # if node already processed
            if v in closed_dict:
                continue

            if v in open_queue:
                # if we get node by lower cost than previous
                if ucost + uv_cost +distance_heuristic(v, goal_key) < open_queue[v]:
                    open_queue[v] = ucost + uv_cost + distance_heuristic(v, goal_key)
                    costs[v]= ucost+uv_cost
                    predecessors[v] = u
            else:
                open_queue[v] = ucost + uv_cost + distance_heuristic(v, goal_key)
                costs[v] = ucost + uv_cost
                predecessors[v] = u
        closed_dict[u] = ucost

    if not goal_found:
        raise ValueError("Goal not found in search.")
    return get_path(origin_key, goal_key, predecessors)



if __name__ == '__main__':
    rospy.init_node('Astar_nav', anonymous=True) 
    initial_1 = rospy.wait_for_message('/ground_truth/state', Odometry)
    goal_1 = rospy.wait_for_message('/move_base_simple/goal', PoseStamped)
    # Converting coordinates from world frame to map frame
    # converting to numpy array and then to a 2d array 
    occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid)
    occupancy_array = np.array(occupancy_grid.data)
    occupancy = np.reshape(occupancy_array,(3328,3328))
    
    # for origin position
    var = initial_1.pose.pose.position
    origin = (int((var.y+p)/r), int((var.x+p)/r))
    print(origin)
    # for goal position
    gol = goal_1.pose.position
    goal23 = (int((gol.y+p)/r), int((gol.x+p)/r))
    print(goal23)
    time.sleep(5)
    publish_path = rospy.Publisher("/publishpath", Path, queue_size=10)
    path = a_star_search(origin, goal23, occupancy)
    path21 = Path()
    path21.header.frame_id="map"
    # convert again to previous coordinates
    for i in path:
        x = i[1]*r - p
        y = i[0]*r - p
        pos=PoseStamped()
        pos.pose.position.x=x
        pos.pose.position.y=y
        path21.poses.append(pos)
        publish_path.publish(path21)
    # print(path21)
    while not rospy.is_shutdown:
        
        
            
        rospy.spin()

