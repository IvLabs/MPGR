#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

import time
from std_srvs.srv import Empty



# /joint1_vel_controller/command
# std_msgs/Float64
# float64 data




if __name__ == '__main__':
    try:
        
        rospy.init_node('sahayak_teleop_node', anonymous=True)

        #declare velocity publisher
        vel_topic_1='/joint1_vel_controller/command'
        velocity_publisher_1 = rospy.Publisher(vel_topic_1, Float64, queue_size=10)

        vel_topic_2='/joint2_vel_controller/command'
        velocity_publisher_2 = rospy.Publisher(vel_topic_2, Float64, queue_size=10)

        vel_topic_3='/joint3_vel_controller/command'
        velocity_publisher_3 = rospy.Publisher(vel_topic_3, Float64, queue_size=10)

        vel_topic_4='/joint4_vel_controller/command'
        velocity_publisher_4 = rospy.Publisher(vel_topic_4, Float64, queue_size=10)


        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()



        time.sleep(2)

        while not rospy.is_shutdown():
            comm = input("Enter move : ")
            #rospy.spin()

            if comm == 'a':
                j1.data = 6.0
                j2.data = 6.0
                j3.data = 6.0
                j4.data = 6.0

            elif comm == 'd':
                j1.data = -6.0
                j2.data = -6.0
                j3.data = -6.0
                j4.data = -6.0
                
            elif comm == 'w':
                j1.data = 6.0
                j2.data = -6.0
                j3.data = 6.0
                j4.data = -6.0
            elif comm == 's':
                j1.data = -6.0
                j2.data = 6.0
                j3.data = -6.0
                j4.data = 6.0
            elif comm == 'x':
                j1.data = 0
                j2.data = 0
                j3.data = 0
                j4.data = 0
            elif comm == 'q':
                j1.data = 0
                j2.data = 0
                j3.data = 0
                j4.data = 0
                break

            velocity_publisher_1.publish(j1)
            velocity_publisher_2.publish(j2)
            velocity_publisher_3.publish(j3)
            velocity_publisher_4.publish(j4)
        




       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")