import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from math import atan2,sqrt
import math
from nav_msgs.msg import Path
import time
from tf.transformations import euler_from_quaternion
import numpy as np


#---------------------------------------------------------------

x = 0.0
y = 0.0 
theta = 0.0
route = []

#---------------------------------------------------------------
def call_back(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

#---------------------------------------------------------------

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

#---------------------------------------------------------------

def dist(a,b):
    return(sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2))

# def normalizeAngle(angle):
#         if angle < -np.pi:
#             angle += 2* np.pi
#         elif angle > np.pi:
#             angle -= 2* np.pi
#         return angle

def normalizeAngle(angle):
        while angle < -np.pi:
            angle += 2* np.pi
        while angle > np.pi:
            angle -= 2* np.pi
        return angle

#---------------------------------------------------------------



if __name__ == '__main__':
    try:
        
        rospy.init_node('path_finder', anonymous=True)

 #---------------------------------------------------------------
        #declare velocity publisher
        vel_topic_1='/joint1_vel_controller/command'
        velocity_publisher_1 = rospy.Publisher(vel_topic_1, Float64, queue_size=10)

        vel_topic_2='/joint2_vel_controller/command'
        velocity_publisher_2 = rospy.Publisher(vel_topic_2, Float64, queue_size=10)

        vel_topic_3='/joint3_vel_controller/command'
        velocity_publisher_3 = rospy.Publisher(vel_topic_3, Float64, queue_size=10)

        vel_topic_4='/joint4_vel_controller/command'
        velocity_publisher_4 = rospy.Publisher(vel_topic_4, Float64, queue_size=10)

        rate= rospy.Rate(5)  

        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        #---------------------------------------------------------------


        # obtaining start position 
        pose_msg =rospy.wait_for_message("/ground_truth/state",Odometry)

        x1 = pose_msg.pose.pose.position.x
        y1 = pose_msg.pose.pose.position.y
        xq,yq,zq,wq = pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w
        roll_x, pitch_y, yaw_z = euler_from_quaternion(xq, yq, zq, wq)

        
        # obtaining goal position
        goal=rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        x2 = goal.pose.position.x
        y2 = goal.pose.position.y

        phi =  atan2(y2-y1,x2-x1) - yaw_z

        phi = normalizeAngle(phi)
        print('Angle', phi)
        #theta= yaw_z

        error = sqrt((x2-x)**2 + (y2-y)**2)
        print('dist', error)

        old_error = 0
        old_phi = 0



        R= 0.08
        L= 0.41

        print('Commencing Operation')

        while not rospy.is_shutdown(): 



            while abs(phi) >= 0.0005 or abs(error) > 0.1:
                rospy.Subscriber("/ground_truth/state", Odometry, call_back)

                #Error in position from goal
                #error = sqrt((x2-x)**2 + (y2-y)**2)
                # print(x,y)
                #print('dist', error)
                # roll_x, pitch_y, yaw_z = euler_from_quaternion(xq, yq, zq, wq)
                # des_x , des_y = point


                # K_v=20
                # K_d= 15
                # K_i= 3

                # # error= sqrt(pow(des_x-x,2)+pow(des_y-y,2)) #Proportional term
                # e_dot= error - old_e #Differential term
                # #print("Error is ", error)
                # #print("Change in error",e_dot)
                # E = E + error #Integral term
                # V= K_v*error + K_d*e_dot 
                # old_e = error

                # print('linear Vel' ,V)

                error = sqrt((x2-x)**2 + (y2-y)**2)
                #print('dist', error)

                phi =  atan2(y2-y1,x2-x1) - theta - np.pi/2 + 0.30
                phi = normalizeAngle(phi)
                # print('Angle', phi)
                print('Angle', phi, '\t', 'Dist',error)

                # differential term
                D_err = error - old_error
                D_phi = phi - old_phi

                # Controller #Not Tuned

                # if error <= 1:
                #     v_des = 0.1 * error
                # else:
                #     v_des = 0.4*error + 0.1*D_err

                v_des = 0.4*error + 0.1*D_err
                w_des = 7*(phi) + 2.3*D_phi

                # update old errors
                old_error = error
                old_phi = phi

                v_r = v_des + L*w_des
                v_l = v_des - L*w_des
                v_l = v_l/R
                v_r = v_r/R

                
                
                # Stop Condition
                if abs(phi)< 0.0005 and (abs(error)< 0.1 or (abs(error)<0.23 and D_err > 0)): 

                    v_l = 0
                    v_r = 0
                    print('done')
                    #break
                
                # Giving Velocity to bot
                j1.data = v_r    
                j2.data = -v_l
                j3.data = v_r
                j4.data = -v_l

                velocity_publisher_1.publish(j1)
                velocity_publisher_2.publish(j2)
                velocity_publisher_3.publish(j3)
                velocity_publisher_4.publish(j4)
                rate.sleep()
            
            # Only Linear vel
            # while abs(error) >= 0.4:
            #     rospy.Subscriber("/ground_truth/state", Odometry, call_back)

            #     error = sqrt((x2-x)**2 + (y2-y)**2)
            #     print('dist', error)

                

            #     # k_t= 5
            #     # k_d= 4
            #         # k_i= 1# Controller
            #     v_des = 0.4*error # Possible bug ############################
            #     w_des = 0

            #     v_r = v_des + L*w_des
            #     v_l = v_des - L*w_des
            #     v_l = v_l/R
            #     v_r = v_r/R

            #     # Select next node
            #     if abs(error)< 0.4: 

            #         v_l = 0
            #         v_r = 0
            #         print('done')
            #         #break
                
            #     # Giving Velocity to bot
            #     j1.data = v_r    # Possible bug ##################################
            #     j2.data = -v_l
            #     j3.data = v_r
            #     j4.data = -v_l

            #     velocity_publisher_1.publish(j1)
            #     velocity_publisher_2.publish(j2)
            #     velocity_publisher_3.publish(j3)
            #     velocity_publisher_4.publish(j4)
            #     rate.sleep()


    
    except rospy.ROSInterruptException:
        pass