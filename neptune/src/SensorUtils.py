#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu


class SensorUtils:
    def __init__(self):
        self.imu_meas, self.encoder_meas, self.pose_measurement = None, [0, 0], None
        imu_rate = 10
        encoder_rate = 10
        lidar_rate = 10 # 0.48

        self.min_imu_time_interval = 1 / imu_rate
        self.min_encoder_time_interval = 1 / encoder_rate
        self.min_lidar_time_interval = 1/ lidar_rate

        self.last_imu_time, self.last_encoder_time, self.last_lidar_time = 0, 0, 0
        rospy.Subscriber("/neptune_measured_pose", Pose, self.measured_pose_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/encoder_twist", Twist, self.encoder_callback)

    def get_imu_and_encoder_meas(self):
        time = rospy.Time.now().to_sec()
        
        if time - self.last_imu_time >= self.min_imu_time_interval:
            self.last_imu_time = time
            return self.imu_meas, self.encoder_meas
        else:
            return None


    # def get_encoder_meas(self):
    #     time = rospy.Time.now().to_sec()
        
    #     if time - self.last_encoder_time >= self.min_encoder_time_interval:
    #         self.last_encoder_time = time
    #         return self.encoder_meas
    #     else:
    #         return None


    def get_pose_measurement(self):
        time = rospy.Time.now().to_sec()
        
        if time - self.last_lidar_time >= self.min_lidar_time_interval:
            self.last_lidar_time = time
            return self.pose_measurement
        else:
            return None


    def imu_callback(self, msg):
        omega = msg.angular_velocity.z
        time = msg.header.stamp.to_sec()

        self.imu_meas = [omega, time] 


    def encoder_callback(self, msg):
        self.encoder_meas = [msg.linear.x, msg.angular.z] 


    def measured_pose_callback(self, msg):
        self.pose_measurement = [msg.position.x, msg.position.y, msg.orientation.z] 

