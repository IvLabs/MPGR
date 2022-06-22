import cv2
import numpy as np
import time
import priority_dict

img = np.zeros((512,512,3), np.uint8)
#img = cv2.circle(img,(200,200), 20, (255,255,255), -1)
img = cv2.circle(img,(300,200), 70, (255,255,255), -1)
img = cv2.circle(img,(200,300), 75, (255,255,255), -1)
#img = cv2.rectangle(img,(184,100),(410,228),(255,255,255),-1)
gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


# start_pt = (100,140)
# end_pt = (270,400)
# #map_img = cv2.flip(gray_img,-1)
# img = cv2.circle(img,start_pt, 5, (0,0,255), -1)
# img = cv2.circle(img,end_pt, 5, (255,0,0), -1)



#img[100,100] = 255
def write_path(image, x, y):
    image = cv2.circle(image, (x,y), radius=0, color=(0, 0, 255))
    return image

def dist(a,b):
    return np.sqrt( (a[0] - b[0])**2 + (a[1] - b[1])**2 )

def hn(node,goal):
    return(dist(node,goal))


def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
        
    return path


def child_node(node,map):
    neighbour = {}

    for i in range(-1,2):
        for j in range(-1,2):
            #print(1)

            x = node[0] + i
            y = node[1] + j

            if not((i,j) == (0,0)) and (y in range(map.shape[1]) and x in range(map.shape[0])):
                
                if map[x,y] == 0:
                    #map[y,x] = 255
                    neighbour[(x,y)] = dist(node,(x,y))
                
                # elif map[y,x] == 255:
                #     neighbour[(y,x)] = 2**30
    
    return neighbour

def a_star_search(origin_key, goal_key, map):
    
    open_queue = priority_dict.priority_dict({})
    
    # The dictionary of closed vertices we've processed.
    closed_dict = []
    
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
        #u = min(open_queue, key = open_queue.get)
        u , u_l = open_queue.pop_smallest()
        
        #print(u)
        #time.sleep(0.5)
        u_cost = costs[u]
        #print(open_queue,'\n')
        #print(predecessors.keys())

        
        
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
            if v in closed_dict :
                continue
            
            
            
            if v not in open_queue :
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
        closed_dict.append(u)
        # open_queue.pop(u)
        # path_now = get_path(origin_key, u, predecessors)
        # pts = np.array(path_now, np.int32)
        # pts = pts.reshape((-1,1,2))
        # img_now = cv2.polylines(map,[pts],False,(255,0,0), 2)
        # cv2.imshow('img',img_now)

        #print(closed_dict)

    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    #return predecessors
    return get_path(origin_key, goal_key, predecessors)



start_pt = (240,100)
end_pt = (400,170)
# start = (100,140)
# end = (100,100)

start = (100,240)
end = (170,400)

# child = child_node(end, thresh_img)
path = [(10, 10), (10, 11), (10, 12), (10, 13), (10, 14), (10, 15), (10, 16), (10, 17), (10, 18), (10, 19), (10, 20)]
#path = a_star_search(start, end, map_img)
path = a_star_search(start, end, gray_img)
#print(path)

# print(child)
# print(path)



#img = cv2.circle(img,(200,200), 20, (255,255,255), -1)
print(gray_img[200,200])
img = cv2.circle(img,start_pt, 5, (0,0,255), -1)
img = cv2.circle(img,end_pt, 5, (255,0,0), -1)

# for i in child:
#     thresh_img[i[0],i[1]] = 255
#     print(i, round(child[i],3), round(hn(i,(1,1)),2))


# pts = np.array(path, np.int32)
# pts = pts.reshape((-1,1,2))
# img = cv2.polylines(img,[pts],False,(255,0,0), 2)
for i in path:
    write_path(img, i[0], i[1])
    write_path(img, i[1], i[0])
# cross = False
# for j in path:
#     if gray_img[j[0],j[1]] == 255:
#         print('white infiltration at ', j)
#         cross = True
#         break
# print(cross)


cv2.imshow('gray_img',gray_img)
#cv2.imshow('map_img',map_img)
cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()