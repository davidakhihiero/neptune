# neptune
A Differential-Drive Mobile Robot

In this project, I built a differential-drive mobile robot, using the ROS framework, for SLAM. The robot used a unicycle-model-based PD controller
to navigate along waypoints computed by an A* algorithm that path-planned using an occupancy grid map. The map was built using lidar
scans. I implemented an Extended Kalman Filter for state estimation that fused lidar data with wheel odometry and IMU data. 

ROS nodes are in Python with one C++ Rosserial node (on the Arduino).

## Nodes:  
**motor_controller_rosserial_node:** C++ rosserial node for low-level control of the motors and measurement of wheel speeds.

**imu_node:** Python node to publish IMU data from the MPU-6050 sensor (I slightly modified this package: https://github.com/OSUrobotics/mpu_6050_driver).

**measure_pose:** Python node to measure pose successive lidar point clouds using the Iterative Closest Point algorithm.

**sensor_utils:** Python node to subscribe to sensor data and publish the data at a controlled rate.

**mapping_node:** Python node to create and update the occupancy grid map and publish it for visualization on RVIZ

**ekf_node:** Python node to estimate the robot's state using the EKF algorithm and publish the state.

**Main_SLAM_node:** Python node to start the SLAM process.


## Launch Files: 
**neptune.launch:** Starts all the nodes (in neptune/src) except the Main_SLAM_node.

**rplidar.launch:** Starts the node for publishing lidar scan data.

## Procedure for SLAM
1. Start the launch file: **roslaunch neptune neptune.launch** 
2. Open RVIZ on another PC that has the same ROS master as the robot and create a visualization for the Map
3. Start the SLAM process: **rosrun neptune RobotControl.py**
4. Stop when satisfied.  


Feel free to create more interesting features :)




