#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math


x, y, theta = 0, 0, 0

def move(speed, distance):
    global x, y
    x0, y0 = x, y

    #define message. Turtlesim uses Twist.
    velocity_message = Twist()
    velocity_message.linear.x = speed

    #define topic and then publisher.
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    distance_moved = 0.0
    t0 = rospy.Time.now().to_sec()

    while distance_moved < distance:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)
        distance_moved = (rospy.Time.now().to_sec() - t0)*speed
#        distance_moved += math.sqrt((x-x0)**2 + (y-y0)**2)
        loop_rate.sleep()

    #stop the robot when distance is reached.
    rospy.loginfo("reached")
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree):
    global theta
    theta0 = theta
    omega =  angular_speed_degree * math.copysign(1, relative_angle_degree)
    
    velocity_message = Twist()
    velocity_message.angular.z = math.radians(omega)

    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
   
    t0 = rospy.Time.now().to_sec()
    cur_angle = 0

    while cur_angle < abs(relative_angle_degree):
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)
        cur_angle = (rospy.Time.now().to_sec() - t0)*angular_speed_degree
        loop_rate.sleep()

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        #Initialize node to the ROS Master
        rospy.init_node('turtlesim_motion_pose', anonymous=True)      
        
        lin_v = 1.0
        ang_v = 30 #deg/s
        m = 0.3

        #Draw M logo
        rotate(ang_v, 45)
        move(lin_v, 4.717*m)    #/
        rotate(ang_v, -45)  
        move(lin_v, 3.0*m)    #_ top
        rotate(ang_v, -90)
        move(lin_v, 2.0*m)    #|
        rotate(ang_v, -90)
        move(lin_v, 1.0*m)   #-
        rotate(ang_v, 90)
        move(lin_v, 4.0*m)    #| long
        rotate(ang_v, 90)
        move(lin_v, 1.0*m)   #-
        rotate(ang_v, -90)
        move(lin_v, 2.0*m)    #|
        rotate(ang_v, -90)
        move(lin_v, 4.0*m)    #_ bottom
        rotate(ang_v, -90)
        move(lin_v, 2.0*m)    #|
        rotate(ang_v, -90)
        move(lin_v, 1.0*m)    #-
        rotate(ang_v, 90)
        move(lin_v, 4.0*m)    #| middle
        rotate(ang_v, 135)
        move(lin_v, 4.717*m)    #/
        rotate(ang_v, -90)
        move(lin_v, 4.717*m)    #\
        rotate(ang_v, 135)
        move(lin_v, 4.0*m)    #| middle
        rotate(ang_v, 90)
        move(lin_v, 1.0*m)   #-
        rotate(ang_v, -90)
        move(lin_v, 2.0*m)    #|
        rotate(ang_v, -90)
        move(lin_v, 4.0*m)    #_ bottom
        rotate(ang_v, -90)
        move(lin_v, 2.0*m)    #|
        rotate(ang_v, -90)
        move(lin_v, 1.0*m)   #-
        rotate(ang_v, 90)
        move(lin_v, 4.0*m)    #| long
        rotate(ang_v, 90)
        move(lin_v, 1.0*m)   #-
        rotate(ang_v, -90)
        move(lin_v, 2.0*m)    #|
        rotate(ang_v, -90)
        move(lin_v, 3.0*m)    #_ top
        rotate(ang_v, -45)
        move(lin_v, 4.717*m)    #\

        

        #rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass

