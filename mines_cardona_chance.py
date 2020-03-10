#!/usr/bin/env python

#Code heavily inspired by the ROS Turtlesim Go to Goal tutorial.
#wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

import rospy, math, time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#Globally keeping track of position
x, y, theta = 0, 0, 0

#Subscriber callback func. Updates position of turtle.
def poseCallback(pose_message):
    global x, y, theta
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
    #print(x, y, theta)


def move2goal(relative_x, relative_y, lin_speed=1.5, ang_speed=6, m = 0.8 ): #m is just scaling
    #define Goal x coordinates
    global x, y, theta
    x0, y0, theta0 = x, y, theta
    goal_x = x0 + relative_x*m
    goal_y = y0 + (relative_y+0.001)*m #0.01 else angle gets off.

    #Define message, topic, and publisher
    vel_msg = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    #Define rate of loop
    loop_rate = rospy.Rate(10)
    i = 0

    #Simple closed loop PID
    while math.sqrt((goal_x - x)**2 + (goal_y - y)**2) > 0.01*m:
        #We want to turn before we begin moving
        if i > 20:
            vel_msg.linear.x = lin_speed * math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        vel_msg.angular.z = ang_speed * (math.atan2(goal_y - y, goal_x - x) - theta)
        i+=1
        rospy.loginfo(vel_msg.angular.z)
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

    #Stop when goal is reached
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin() #for ctrl-C
        



if __name__ == '__main__':
    try:
        #Initialize node to the ROS Master
        rospy.init_node('turtlesim_motion_pose', anonymous=True)      
        
        #declare pose subscriber
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 

        time.sleep(2)

        #Draw M logo

        move2goal(2.5, 4)
        move2goal(3, 0)
        move2goal(0, -2)
        move2goal(-1, 0)
        move2goal(0, -4)
        move2goal(1, 0)
        move2goal(0, -2)
        move2goal(-4, 0)
        move2goal(0, 2)
        move2goal(1, 0)
        move2goal(0, 4)
        move2goal(-2.5, -4)
        move2goal(-2.5, 4)
        move2goal(0, -4)
        move2goal(1, 0)
        move2goal(0, -2)
        move2goal(-4, 0)
        move2goal(0, 2)
        move2goal(1, 0)
        move2goal(0, 4)
        move2goal(-1, 0)
        move2goal(0, 2)
        move2goal(3, 0)
        move2goal(2.5, -4)

        #rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass

