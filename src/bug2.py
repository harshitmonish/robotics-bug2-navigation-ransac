#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import random
import math
import tf
from tf.transformations import euler_from_quaternion

#state GOALSEEK = 1 ; WALLFOLLOW = 0
state = 1
dataR = None

def calc_line_dist(x, y):
    global initial_x
    global initial_y
    global goal_x
    global goal_y

    return (np.abs(((goal_y - initial_y)*x) - ((goal_x - initial_x)*y) + (goal_x * initial_y) - (goal_y * initial_x)) / math.sqrt(np.power((goal_y - initial_y), 2) + np.power((goal_x - initial_x), 2)))

def calc_goal_dist(x, y):
    global goal_x
    global goal_y
    return (math.sqrt((y - goal_y)*(y - goal_y) + (x - goal_x)*(x - goal_x)))

def calc_forward_vel(data):
    #taking forward angle to left and right which maps to 240-480
    min_val = min(data[240:480])
    if min_val < 0.75:
        return 0
    else:
        return 1

def calc_rotation_goalseek(angle):
    if abs(angle) > np.pi/30:
        return angle
    else:
        return 0

def calc_rotation_wallfollow(data):
    right = data[180:360]
    left = data[360:450]

    min_right = min(right)
    min_left = min(left)

    if min_right < 0.1 and min_left < 0.1:
        return np.pi/2
    else:
        t = (np.pi/2 - ((np.pi/6)*(3 - min_right)))
        return t

def detect_obstacle(angle, data):
    global state

    if min(data[240:480]) < 0.75:
        return 1
    else:
        return 0

def goal_achieved(distance):
    if distance < 1:
        return 1
    else:
        return 0


def bug2_func(msg):
    global state
    global dataR

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = 0
    if dataR == None:
        return 
    #distance of robot from start-goal line
    lineDist = calc_line_dist(x, y)

    #distance of robot from goal
    goalDist = calc_goal_dist(x, y)

    roll, pitch, yaw = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

    cmd_vel = Twist()

    goal_angle = (math.atan2((9.0 - y), (4.5 - x)) - yaw)
    cmd_vel.linear.x = calc_forward_vel(dataR)

    #if GOALSEEK
    if state == 1:
        cmd_vel.angular.z = calc_rotation_goalseek(goal_angle)
    #if WALLFOLLOW
    if state == 0:
        cmd_vel.angular.z = calc_rotation_wallfollow(dataR)

    #changing state depending on the condition
    if (detect_obstacle(goal_angle, dataR) and state == 1):
        print("State Changed to WALLFOLLOW")
        state = 0
    elif (detect_obstacle(goal_angle, dataR) == 0 and state == 0):
        print("State Changed to GOALSEEK")
        state = 1

    #checking if goal is achieved
    if goal_achieved(goalDist):
        #stop
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        print("Goal Reached")

    pub = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size = 10)
    pub.publish(cmd_vel)

def callback(data):
    global dataR
    dataR = data.ranges

def main():
    global goal_x
    global goal_y
    global initial_x
    global initial_y

    #fetching initial and final poition
    initial_x = rospy.get_param("/lab3/initial_x")
    initial_y = rospy.get_param("/lab3/initial_y") 
    goal_x = rospy.get_param("/lab3/goal_x")
    goal_y = rospy.get_param("/lab3/goal_y")

    rospy.init_node("bug2")
    rate = rospy.Rate(5)
    rospy.Subscriber("/front/scan", LaserScan, callback)

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("/husky_velocity_controller/odom", Odometry, bug2_func, queue_size=10)
            rate.sleep()
        except:
            continue




if __name__ =="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass






