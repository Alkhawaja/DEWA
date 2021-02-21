Odometry, IMU and GPS - EKF Localizatiion

Download the .bag file and rename it to ekf.bag

1 Run roscore

2 Run rosbag play ekf.bag /imu/data:=/imu_data /warthog_velocity_controller/odom:=/odom

3 Run rosrun gps_common utm_odometry_node

4 Run roslaunch robot_pose_ekf example_with_gps.launch

Rostopic list

/odom

/gps

/imu

/robot_pose_ekf/odom_combined

before catkin_make make sure to

sudo apt-get install ros-indigo-can-*

sudo apt-get install ros-indigo-geographic-msgs

sudo apt-get install libgps-dev

sudo apt-get install ros-indigo-joint-state-publisher-gui (change cmake to 2.8.10.2 in CMakelists)

You may use rviz to check the results
