
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import priority_dict

#---------------------------------------------------------------------
# map
pgm1 = cv2.imread("my_world_map.pgm",1)
dims = pgm1.shape
pgm = pgm1[1200:2100,1470:1850]

print(pgm.shape, dims[0]//4 + 400, 3*dims[0]//4 - 450, 2*dims[1]//5, 3*dims[1]//5 )

gray_img = cv2.cvtColor(pgm,cv2.COLOR_BGR2GRAY)  # for A* calculation

# kernel = np.ones((3,3),np.uint8)
# #closing = cv2.morphologyEx(gray_img, cv2.MORPH_CLOSE, kernel)
# #dilation = cv2.dilate(gray_img,kernel,iterations = 1)
# blur_img = cv2.GaussianBlur(gray_img,(5,5),0)
ret,thresh_img = cv2.threshold(gray_img,240,255,cv2.THRESH_BINARY)
map_img = cv2.medianBlur(thresh_img,5)

#---------------------------------------------------------------------
# euclidean distance 
def dist(a,b):
    return round(np.sqrt( (a[0] - b[0])**2 + (a[1] - b[1])**2 ),4)

# heuristic function - euclidean distance from goal
def hn(node,goal):
    return(dist(node,goal))

# Get an array of path coordinates
def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
        
    return path

# Get all possible valid child nodes of given node
def child_node(node,map):
    neighbour = {}

    for i in range(-1,2):
        for j in range(-1,2):
            #print(1)

            y = node[0] + i
            x = node[1] + j

            if not((i,j) == (0,0)) and (y in range(map.shape[0]) and x in range(map.shape[1])):
                
                if map[y,x] == 255:
                    #map[y,x] = 255
                    neighbour[(y,x)] = dist(node,(y,x))
                # elif map[y,x] == 255:
                #     neighbour[(y,x)] = 999999
    
    return neighbour
#---------------------------------------------------------------------
# The A* Algorithm
def a_star_search(origin_key, goal_key, map):
    
    open_queue = priority_dict.priority_dict({})
    
    # The dictionary of closed vertices we've processed.
    closed_dict = {}
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    
    # The dictionary that stores the best cost to reach each
    # vertex found so far.
    costs = {}
    
    
    # Add the origin to the open queue and the costs dictionary.
    costs[origin_key] = 0.0
    open_queue[origin_key] = hn(origin_key, goal_key)
    #print(open_queue)
    #u = min(open_queue, key = open_queue.get)


    goal_found = False
    #return child_node(u , map)
    while (len(open_queue) != 0):
        u , u_l = open_queue.pop_smallest()
        
        #print(u)
        #time.sleep(2)
        u_cost = costs[u]
        # print(open_queue)
        
        
        # if goal is reached break the loop
        if u == goal_key:
            goal_found = True
            break
        
        
        # iterate over the neighbouring vertices
        child_nodes = child_node(u,map)
        #print(child_nodes)
        for edge in child_nodes:
            
            # Get the edge and edgeweight
            v , uv_cost = edge , child_nodes[edge]
            
            # Check if v is in the closed dict
            if v in closed_dict:
                continue
            
            
            
            if v not in open_queue:
                open_queue[v] = u_cost + uv_cost + hn(v, goal_key)
                costs[v] = u_cost + uv_cost
                predecessors[v] = u
            else:
                v_cost_initial = open_queue[v]
                if v_cost_initial > u_cost + uv_cost + hn(v, goal_key):
                    open_queue[v] = u_cost + uv_cost + hn(v, goal_key)
                    costs[v] = u_cost + uv_cost
                    predecessors[v] = u
        
        # add vertex in closed dict after all possible neighbours are evaluated
        closed_dict[u] = True
        #print(closed_dict)

    if not goal_found:
        print('not found')
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    #return predecessors
    return get_path(origin_key, goal_key, predecessors)


#---------------------------------------------------------------------
start = (100,230)
end = (100,341)

# child = child_node(start, img)
path = a_star_search(start, end, map_img)

start = (start[1],start[0])
end = (end[1],end[0])
#path = [(10, 10), (10, 11), (10, 12), (10, 13), (10, 14), (10, 15), (10, 16), (10, 17), (10, 18), (10, 19), (10, 20)]
img = cv2.circle(pgm,start, 5, (0,0,255), -1)
img = cv2.circle(pgm,end, 5, (255,0,0), -1)
# img = cv2.circle(map_img,start, 5, (0,0,255), -1)
# img = cv2.circle(map_img,end, 5, (255,0,0), -1)
print('Done')


pts = np.array(path, np.int32)
pts = pts.reshape((-1,1,2))
img = cv2.polylines(pgm,[pts],False,(0,255,0), 1)

cv2.imshow('map1',pgm)
cv2.imshow('map_thresh',map_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
