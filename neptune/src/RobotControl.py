#!/usr/bin/env python

# Main node for SLAM

from traceback import print_tb
import rospy
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
from shortest_path import a_star
from DiffDriveController import DiffDriveController

# Robot's initial pose is set to 0, 0, 0
pose = [0, 0, 0]

occ_grid = np.full([250, 250], 0)

# Maximum and minimum velocities of the robot
max_v, max_omega, min_v, min_omega = 0.3, 0.3, 0.1, 0.1

controller = DiffDriveController(max_v, max_omega, min_v, min_omega)
vel_pub = vel_pub = rospy.Publisher("/motor_command", Twist, queue_size=10)
lidar_ranges = None
lidar_angles = None


def lidar_callback(msg):
    global lidar_ranges, lidar_angles
    ranges = msg.ranges
    angles = []
    angle_increment = msg.angle_increment
    angle = msg.angle_min
    angle_max = msg.angle_max

    while angle <= angle_max + angle_increment:
        angles.append(angle)
        angle += angle_increment
    

    new_ranges = []
    new_angles = []
    for i in range(len(ranges)):
        if not (np.isneginf(ranges[i]) or np.isposinf(ranges[i])):
            new_ranges.append(ranges[i])
            new_angles.append(angles[i])
    
    lidar_ranges = new_ranges
    lidar_angles = new_angles


def pose_callback(msg):
    global pose
    pose = [msg.position.x, msg.position.y, msg.orientation.z]


def map_callback(msg):
    global occ_grid
    data = np.array(msg.data)
    occ_grid = data.reshape([250, 250])


def get_goal():
    """
    Function to randomly choose a goal point from the map
    :return: A goal pose
    """
    global pose, occ_grid, lidar_ranges, lidar_angles
    x, y, theta = 0, 0, 0

    while lidar_ranges is None:
        pass

    max_dist = max(lidar_ranges)
    # print(max_dist)
    angle = lidar_angles[lidar_ranges.index(max_dist)]
    goal_dist = max_dist * 0.5
    x = goal_dist * np.cos(pose[2] + angle)
    y = goal_dist * np.sin(pose[2] + angle)

    return [x, y, theta]


def stop():
    """
    Function to command the robot to stop
    """ 
    global vel_pub
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    vel_pub.publish(msg)


def slam():
    """
    SLAM function
    Repeatedly choose a goal point from the map, path-plan to the goal and control the robot to the goal via waypoints
    """
    global pose, vel_pub

    while True and not rospy.is_shutdown():
        start = pose
        goal = get_goal()
        path = a_star(occ_grid, start, goal)

        while len(path) == 0 and not rospy.is_shutdown():
            goal = get_goal()
            path = a_star(occ_grid, start, goal)
        
        print(path)
        step_size = 12

        rate = rospy.Rate(10)

        for i in range(0, len(path), step_size):
            start_i = pose
            goal_i = path[i]
            done = False

            while not done and not rospy.is_shutdown():
                start_i = pose
                print (pose)
                v, omega, done = controller.compute_velocity(start_i, goal_i)
                twist_msg = Twist()
                twist_msg.linear.x = v
                twist_msg.angular.z = omega
                vel_pub.publish(twist_msg)
                rate.sleep()

        # Make sure robot moves to last waypoint irrespective of step_size
        start_i = pose
        goal_i = path[-1]

        done = False

        while not done and not rospy.is_shutdown():
            print (pose)
            start_i = pose
            v, omega, done = controller.compute_velocity(start_i, goal_i)
            twist_msg = Twist()
            twist_msg.linear.x = v
            twist_msg.angular.z = omega
            vel_pub.publish(twist_msg)
            rate.sleep()
        stop()


if __name__ == "__main__":
    rospy.init_node("Main_SLAM_node", anonymous=True)
    rospy.Subscriber("scan", LaserScan, lidar_callback)
    rospy.Subscriber("/neptune_EKF_pose", Pose, pose_callback)
    rospy.Subscriber("/ogrid", OccupancyGrid, map_callback)
    rospy.on_shutdown(stop)
    slam()
    stop()
    