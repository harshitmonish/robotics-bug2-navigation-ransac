#!/usr/bin/env python
import rospy
import random
import numpy as np
from numpy.linalg  import norm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
import math
from sensor_msgs.msg import LaserScan

points_x = []
points_y = []

rate = 10

ransac_x = np.array([])
ransac_y = np.array([])

# function to implement ransac algo on laser output
def ransac(a, b):
    global ransac_x
    global ransac_y
    ransac_iters = 50
    ransac_dist_threshold = 0.05
    ransac_points_threshold = 5

    a = ransac_x
    b = ransac_y
    #callback function of laser output
    def callback(range):
        global ransac_x
        global ransac_y
        values = np.array(range.ranges)
    
        for idx, val in enumerate(values):
            if val == float("inf"):
                values[idx] = 30
            if val == float("-inf"):
                values[idx] = -30
        a = values *sins
        b = values * coss
        ransac_x = a.reshape((a.shape[0], 1))
        ransac_y = b.reshape((b.shape[0], 1))

    rospy.Subscriber("/front/scan", LaserScan, callback)

    if len(a) == 0:
        return

    global points_x
    global points_y

    points_x= [-1]
    points_y= [-1]

    max_x = -1
    max_y = -1
    min_x = 10**9
    min_y = 10**9

    inliers = []
    outliers = []

    indices = range(len(a))
    for i in range(ransac_iters):
        temp_inliers = []
        temp_outliers = []

        #picking 2 random indices
        p1 = random.randint(0, len(indices)-1)
        p2 = random.randint(0, len(indices)-1)

        if p1 == p2 or (a[indices[p1]] == 0) or (a[indices[p2]] == 0):
            continue
        p1x = a[indices[p1]]
        p1y = b[indices[p1]]
        p2x = a[indices[p2]]
        p2y = b[indices[p2]]

        for i in indices:
            if a[i] == 0 or b[i] == 0:
                continue
            ai = a[i]
            bi = b[i]
            #calculating distance
            distance = np.abs(((p2y - p1y)* ai) - ((p2x - p1x)*bi) + (p2x * p1y) - (p2y * p1x)) / math.sqrt(np.power((p2y - p1y), 2) + np.power((p2x - p1x), 2))

            if distance < ransac_dist_threshold:
                temp_inliers.append(i)
            else:
                temp_outliers.append(i)
        #if found better inliers, update
        if len(inliers) < len(temp_inliers):
            inliers = temp_inliers
            outliers = temp_outliers
            max_x = np.max(ransac_x[inliers])
            max_y = np.max(ransac_y[inliers])
            min_x = np.min(ransac_x[inliers])
            min_y = np.min(ransac_y[inliers])
    #updating inliers
    if len(inliers) > ransac_points_threshold:
        points_x.append(max_x)
        points_y.append(max_y)
        points_x.append(min_x)
        points_y.append(min_y)


def main():
    rospy.init_node('ransac')
    r = rospy.Rate(rate)
    
    publisher_node1 = rospy.Publisher("/ransac_visualization", Marker, queue_size=10)
    publisher_node2 = rospy.Publisher("/laser_visualization", Marker, queue_size=10)
    global sins, coss, points_x, points_y, ransac_x, ransac_y

    laser_op = np.linspace(2.356, -2.356, 720)
    sins = np.sin(laser_op)
    coss = np.cos(laser_op)
    a = 0
    b = 0
    while not rospy.is_shutdown():
        try:
            ransac(a, b)
            marker1 = Marker()
            marker1.header.frame_id = "front_laser"
            marker1.header.stamp = rospy.Time.now()
            marker1.type = Marker.LINE_STRIP
            marker1.action = Marker.ADD
            marker1.scale.x = 0.2
            marker1.color.g = 1.0
            marker1.color.r = 0.0
            marker1.color.b = 0.0
            marker1.color.a = 1.0

            marker2 = Marker()
            marker2.header.frame_id = "front_laser"
            marker2.header.stamp = rospy.Time.now()
            marker2.type = Marker.LINE_STRIP
            marker2.action = Marker.ADD
            marker2.scale.x = 0.2
            marker2.color.b = 1.0
            marker2.color.r = 0.0
            marker2.color.g = 0.0
            marker2.color.a = 1.0

            n = len(points_x)
            for i in range(1, n):
                p = Point()
                p.x = points_x[i]
                p.y = points_y[i]
                marker1.points.append(p)

            for i in range(720):
                if len(ransac_x) == 0 or (ransac_x[i] == 0 and ransac_y[i] == 0):
                    continue

                p = Point()
                p.x = ransac_x[i][0]
                p.y = ransac_y[i][0]
                marker2.points.append(p)
            publisher_node1.publish(marker1)
            publisher_node2.publish(marker2)

            r.sleep()
        except:
            continue

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

