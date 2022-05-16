# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 10:32:32 2022

@author: dheeraj
"""

import priority_dict
from math import dist,sqrt


def get_nearest_pixels(node,image):
    pix_list=[]
    y,x = node
    for i in range(-1,2):
        for j in range(-1,2):
            t1,t2= y+i,x+j
            pixel=image[t1,t2]
            if t1==y and t2==x:
                continue
            elif pixel==0:
                p= [y,x]
                q= [t1,t2]
                length= dist(p, q)
                
                pix_list.append((t1,t2,'%.2f'%length))
            elif pixel==100:
                length = 100000
            elif pixel== -1:
                continue
    return pix_list

# This function follows the predecessor
# backpointers and generates the equivalent path from the
# origin as a list of vertex keys.
def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
        
    return path


def distance_heuristic(current_node,goal_node):
    y1,x1 = current_node
    y2,x2 = goal_node
    d= sqrt((y2-y1)**2 + (x2-x1)**2)
    d= float('%.3f'%d)
    return d

def a_star_search(origin_key, goal_key, image):
    # The priority queue of open vertices we've reached.
    # Keys are the vertex keys, vals are the accumulated
    # distances plus the heuristic estimates of the distance
    # to go.
    open_queue = priority_dict.priority_dict()
    
    # The dictionary of closed vertices we've processed.
    closed_dict = {}
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    predecessors[origin_key]= 0
    
    # The dictionary that stores the best cost to reach each
    # vertex found so far.
    costs = {}
    
    
    # Add the origin to the open queue and the costs dictionary.
    costs[origin_key] = 0.0
    open_queue[origin_key] = distance_heuristic(origin_key, goal_key)

    
    goal_found = False
    while (open_queue):
        u=open_queue.pop_smallest()
        #u=(U[0],U[1])
        #u_val=U[1]
        uCost= costs[u]
        
        if u==goal_key: 
            goal_found= True
            break
            
        
        for edge in get_nearest_pixels(u,image):
            v=(edge[0], edge[1])
            uvCost = float(edge[2])
            h_v= distance_heuristic(v, goal_key)
            
            if v in closed_dict:
                continue
            
            if v in open_queue:
                if uCost+uvCost+h_v< open_queue[v]:
                    open_queue[v]= uCost+uvCost+h_v
                    costs[v]= uCost+uvCost
                    predecessors[v]=u
            
            else:
                open_queue[v]= uCost+uvCost+h_v
                costs[v]= uCost+uvCost
                predecessors[v]=u
                

        
        closed_dict[u]=uCost
    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)   