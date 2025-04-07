# ros2_rover
ros2 rover with autonamus capability and vision based navigation using ros2 with onboard compuer and arduino 

## [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)
Driver to use ROS2 for [SLAMTEC](https://www.slamtec.com/en) LIDAR. all instructions needed are writen in the very link. It's worth to mention that you need to publish tf yourself. It can be used in slams like gmapping, slam_toolbox and Cartographer.
To run Rtabmap change :
```
mkdir -p ws_rover/src && cd ws_rover/
git clone https://github.com/Slamtec/sllidar_ros2 ./src
source /opt/ros/$ROS_DISTRO/setup.bash && colcon buld --symlink-install
source ./install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
```
## using Rtabmap for odometry with just lidar
use fallowing commands for now:
```
/opt/ros/$ROS_DISTRO/share/rtabmap_demos/launch/turtlebot3/turtlebot3_scan.launch.py
'frame_id':'base_link'
localization', default_value='true'

ros2 run rtabmap_odom icp_odometry
ros2 launch rtabmap_demos turtlebot3_scan.launch.py
```