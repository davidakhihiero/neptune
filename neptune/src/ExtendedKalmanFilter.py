#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist
from SensorUtils import SensorUtils

# Node for the Robot's state estimation using an Extended Kalman Filter 
# Note 1: A particle filter would probably be easier *sighs*
# Note 2: I tried a particle filter, it was annoying and frustrating lol :)

sensor_utils = SensorUtils()

last_time = None  # Used to keep track of time between measurements 
Q_t = np.array([[0.01,  0],
                [0,     0.005]]) 
R_t = np.array([[0.04,  0,     0],
                [0,      0.02, 0],
                [0,      0,     0.01]])
pose = np.zeros(3)
robot_pose = Pose()
P_t = 1e6 * np.eye(3)
encoder_meas, imu_meas, measured_pose = None, None, None
imu_factor = 1 # 0.5


def prediction(v, omega, current_time):
    """
    Function to implement the prediction step of the EKF
    :param v: The robot's linear velocity in m/s
    :param omega: The robot's angular velocity in rad/s
    :param current_time: The current system time in seconds
    """
    global last_time, pose, P_t, Q_t

    if last_time is None:
        last_time = current_time
        return
    
    u_t_hat = pose + (current_time - last_time) * np.array([v * np.cos(pose[2]),
                                                            v * np.sin(pose[2]),
                                                            omega])
    
    df_dx = np.eye(3) + (current_time - last_time) * np.array([[0, 0, -v * np.sin(pose[2])],
                                                               [0, 0,  v * np.cos(pose[2])],
                                                               [0, 0,  0]])
    
    df_dn = (current_time - last_time) * np.array([[np.cos(pose[2]), 0],
                                                  [np.sin(pose[2]),  0],
                                                  [0,                1]])

    P_t_hat = np.dot(np.dot(df_dx, P_t), df_dx.T) + \
        np.dot(np.dot(df_dn, Q_t), df_dn.T)

    pose = u_t_hat
    P_t = P_t_hat
    
    last_time = current_time


def update(measured_pose):
    """
    Function to implement the update step of the EKF
    :param measured_pose: The robot's pose as "measured" by the lidar
    """
    global pose, P_t, R_t

    dh_dx = np.eye(3)

    k = np.dot(np.dot(P_t, dh_dx.T),
                np.linalg.inv((np.dot(np.dot(dh_dx, P_t), dh_dx.T) + R_t)))

    pose = np.reshape(pose, [3, 1]) + np.dot(k, (np.reshape(measured_pose, [3, 1]) - np.reshape(pose, [3, 1])))
    P_t = P_t - np.dot(k, np.dot(dh_dx, P_t))

    pose = np.reshape(pose, [1, 3])[0]
    P_t = P_t


def step_filter(encoder_meas, imu_meas, measured_pose):
    """
    Function to implement the EKF
    :param encoder_meas: The measurement from the wheel encoder sensor for v and omega
    :param imu_meas: The measurement from the IMU for the z-angular velocity and the current system time
    :param measured_pose: The pose as measured from successive lidar point clouds using the ICP algorithm 
    """
    global pose, imu_factor

    if encoder_meas is None and imu_meas is None and measured_pose is None:
        return
    elif measured_pose is not None and imu_meas is None:
        update(measured_pose)
    elif measured_pose is None and imu_meas is not None:
        v = encoder_meas[0]
        omega = (encoder_meas[1] * (1 - imu_factor)) + (imu_meas[0] * imu_factor) 
        current_time = imu_meas[1] 
        prediction(v, omega, current_time)
    elif measured_pose is not None and imu_meas is not None and encoder_meas is not None:
        v = encoder_meas[0]
        omega = (encoder_meas[1] * (1 - imu_factor)) + (imu_meas[0] * imu_factor) 
        current_time = imu_meas[1] 
        prediction(v, omega, current_time)
        update(measured_pose)




if __name__ == "__main__":
    rospy.init_node("EKF_node", anonymous=True)
    rospy.wait_for_message("/encoder_twist", Twist)
    pose_publisher = rospy.Publisher("/neptune_EKF_pose", Pose, queue_size=10)

    rate = rospy.Rate(10)

    while True and not rospy.is_shutdown():
        imu_meas, encoder_meas = sensor_utils.get_imu_and_encoder_meas()
        measured_pose = sensor_utils.get_pose_measurement()

        step_filter(encoder_meas, imu_meas, measured_pose)
        robot_pose.position.x = pose[0]
        robot_pose.position.y = pose[1]
        robot_pose.orientation.z = pose[2]

        pose_publisher.publish(robot_pose)
        rate.sleep()



