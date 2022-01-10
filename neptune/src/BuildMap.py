#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point
from occupancy_grid_mapping import occupancy_grid_mapping
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Node to build the occupancy grid map and publish it

map_ = np.zeros([250, 250])
map_pub = rospy.Publisher("/ogrid", OccupancyGrid, queue_size=10)
meta_data = MapMetaData()
meta_data.resolution = 0.04
meta_data.width = 250
meta_data.height = 250
origin = Pose()
origin.position.x = -5
origin.position.y = -5
meta_data.origin = origin

occgrid = OccupancyGrid()
occgrid.info = meta_data

pose_ = [0, 0, 0]
lidar_ranges = [0]
lidar_angles = [0]

rate = None


def pub_map(ogm):
    """
    Function to publish the occupancy grid map
    :param ogm: The 2D occupancy grid map
    """
    global meta_data, occgrid, map_pub, pose_, origin

    size = 62500 # 250 * 250
    data = np.zeros(size, dtype=int)
    k = 0
    for i in range(250):
        for j in range(250):
            if ogm[i][j] == 0:
                data[k] = -1
                k += 1
            else:
                x = ogm[i][j]
                data[k] = int(round(((x + 100) / 200) * (100)))
                k += 1

    occgrid.data = data
    map_pub.publish(occgrid)


def lidar_callback(msg):
    global map_, pose_, lidar_ranges, lidar_angles
    ranges = msg.ranges
    angles = []
    angle_increment = msg.angle_increment
    angle = msg.angle_min
    angle_max = msg.angle_max

    while angle <= angle_max + angle_increment:
        angles.append(angle)
        angle += angle_increment
    

    lidar_ranges = []
    lidar_angles = []

    for i in range(len(ranges)):
        if not (np.isneginf(ranges[i]) or np.isposinf(ranges[i])):
            lidar_ranges.append(ranges[i])
            lidar_angles.append(angles[i])

    #rate.sleep()


def pose_callback(msg):
    global pose_
    x = msg.position.x
    y = msg.position.y
    theta = msg.orientation.z

    pose_ = [x, y, theta]


def start():
    global rate, map_, pose_
    rospy.init_node("mapping_node", anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/neptune_EKF_pose", Pose, pose_callback)
    rospy.Subscriber("scan", LaserScan, lidar_callback)
    
    while True and not rospy.is_shutdown():
        map_ = occupancy_grid_mapping(occ_grid_map=map_, ranges=lidar_ranges, scan_angles=lidar_angles, pose=pose_)
        pub_map(map_)
        rate.sleep()
  
if __name__ == "__main__":
    start()
