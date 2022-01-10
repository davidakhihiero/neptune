#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import time
from iterative_closest_point import iterative_closest_point

old_points = []
new_points = []
old_pose = [0, 0, 0]
waiting = False
x = 0
t = 0
pose_publisher = None

def callback(msg):
    global old_points, new_points, old_pose
   
    ranges = msg.ranges
    angles = []
    angle_increment = msg.angle_increment
    angle = msg.angle_min
    angle_max = msg.angle_max

    while angle <= angle_max + angle_increment:
        angles.append(angle)
        angle += angle_increment
    

    if len(old_points) == 0:
        for i in range(len(ranges)):
                if not (np.isneginf(ranges[i]) or np.isposinf(ranges[i])):
                    old_points.append((ranges[i] * np.cos(angles[i]), ranges[i] * np.sin(angles[i])))
    elif len(new_points) == 0:
        for i in range(len(ranges)):
            if not (np.isneginf(ranges[i]) or np.isposinf(ranges[i])): 
                new_points.append((ranges[i] * np.cos(angles[i]), ranges[i] * np.sin(angles[i])))

    if len(new_points) > 0:
        new_pose = iterative_closest_point(old_pose, old_points, new_points)
        pose = Pose()
        pose.position.x = new_pose[0]
        pose.position.y = new_pose[1]
        pose.orientation.z = new_pose[2]
        pose_publisher.publish(pose)

        old_pose = new_pose
        old_points = new_points
        new_points = []
       
    

def start():
    global pose_publisher
    rospy.init_node("measured_pose_node", anonymous=True)
    pose_publisher = rospy.Publisher("/neptune_measured_pose", Pose, queue_size=1)
    rospy.Subscriber("scan", LaserScan, callback, queue_size=1)
    rospy.spin()
    

if __name__ == "__main__":
    start()
