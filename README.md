# Autonomous Tunnel Driving package

This package is for running mobile robot under tunnel environment.

### Used Sensors are
* 45-degree tilted 2D Lidar: for aisle-follow driving 
* 3D LiDAR on mobile robot: for detecting obstacles, mapping, localization

### Consisting Nodes are
* #### Aisle detect Node
    * Subscribe
        * /rp/scan (sensor_msgs/LaserScan)
    * Publish: 
        * /cluster_line (sensor_msgs/PointCloud2): detected aisle
        * /aisle_points (sensor_msgs/PointCloud2): robot's direction on line, line center, line start, line end
     * Parameters
        * line_thresh (defalt 0.5): Ransac threshold of line model
        * aisle_width (defatl 0.6): Horizontal width to be cropped from lidar scan

* #### Obstacles Node
    * Subscribe
        * /velodyne_points (sensor_msgs/PointCloud2)
    * Publish: 
        * /cropped_obs (sensor_msgs/PointCloud2): detected obstacles within the aisle zone
        * /obs_dists (std_msgs/Float32MultiArray): Right and Left obstacles's distances 
     * Parameters
        * obs_x_min / obs_x_max (default 0.0 / 1.0): x range of obstacle detect region (m)
        * obs_y_min / obs_y_max (default -0.3 / 0.3): y range of obstacle detect region (m)
        * obs_z_min / obs_z_max (default -0.1 / 0.5): z range of obstacle detect region (m)

* #### Localization Node
    * Subscribe
        * /amcl_pose (geometry_msgs/PoseWithCovarianceStamped): amcl_pose topic can be published by running [amcl package](http://wiki.ros.org/amcl) simultaneously.
        * /move_base_simple/goal (geometry_msgs/PoseStamped): The target position can be set with __2D Nav Goal__ of Rviz.
    * Publish: 
        * /localization_data (std_msgs/Float32MultiArray): [0]-global dist error, [1]-global angle error, [2]-arrival flag, [3]-rotating flag
     * Parameters
        * global_dist_boundary (default 0.3): Global distance boundary error to arrival (m)
        * global_ang_boundary (default 0.05): Global angular boundary error to arrival (rad)

* #### Cmd Puslish Node
    * Subscribe (data from other nodes)
        * /joystick (sensor_msgs::Joy): amcl_pose topic can be published by running [amcl package](http://wiki.ros.org/amcl) simultaneously.
        * /obs_dists (std_msgs/Float32MultiArray)
        * /aisle_points (sensor_msgs/PointCloud2)
        * /localization_data (std_msgs/Float32MultiArray)
        * /lidar_driving (std_msgs/Bool) : Should be puslished manually to execute the /cmd_vel publishing function.
    * Publish: 
        * /cmd_vel (geometry_msgs/Twist): Final cmd_vel combining information from multiple nodes
    * Parameters
        * Kpy_param (default 1.1): Rotation control gain for aisle driving
        * Kpy_param_rot (default 0.01): Rotation control gain for static rotation
        * linear_vel (default 0.0): Linear velocity of mobile robot (m/s)
        * robot_width (default 0.45): Robot width (m)
        *  obs_coefficient (default 0.5): Obstacle avoidance control ratio
        * front_obs(default 0.6): Distance of front obstacles to be avoided 
        * boundary_percent (default 0.02): Percentage of free width on both sides of the aisle for safe driving 
        * spare_length (default 1.5): Additional driving after avoiding obstacles
        * amcl_driving (default false): Whether to check the location of mobile robot
        * check_obstacles (default false): Whether to check for obstacles

### Required ROS Package
* slam-gmapping
* amcl 
```bash
sudo apt-get install ros-melodic-slam-gmapping
sudo apt-get install ros-melodic-amcl
```

### Launch files
* only_driving.launch : driving using scan data with no mapping and localization
* mapping.launch : Generates map with autonomous driving (gmapping ros package is included)
* amcl_driving.launch : Round trip driving with localization (amcl ros package is included)
* simulation.launch : run with bag file

### parameters for gmapping and amcl
<Overall parameter>
* update_min_d: 파티클을 업데이트 하기위해 필요한 translation movement (작을 수록 포즈 계산을 자주 한다)
* update_min_a: 티클을 업데이트 하기위해 필요한 rotational movement 
* min_particles / max_particles: 많을 수록 신뢰도가 높아지지만 계산 처리가 늦어질 수 있음

 

<Odometry model parameter>

노이즈를 키울 수록 파티클 샘플링을 넓은 영역에서 한다
* odom_alpha1: rotation에 대한 rotation 노이즈 (로봇이 회전을 했을 때 생기는 rotation 노이즈) 
* odom_alpha2: tranlation에 대한 rotation 노이즈 (로봇이 회전을 했을 때 생기는 translation 노이즈)
* odom_alpha3: translation에 대한 translation 노이즈
* odom_alpha4: rotation에 대한 rotation 노이즈

__Running__
```bash
roslaunch leo_driving only_driving.launch
roslaunch leo_driving mapping.launch
roslaunch leo_driving amcl_driving.launch
roslaunch leo_driving simulation.launch
```
