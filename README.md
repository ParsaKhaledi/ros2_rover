# ros2_rover
ros2 rover with autonamus capability and vision based navigation using ros2 with onboard compuer and arduino. Based on microservice arciture and usage of docker, this repo contains docker-compose file to split various parts of autonamus stack as sperate containers. For docker image size reduction, all components are placed in same docker image and with different startup commands. X11 is used for visualization.
## Use Docker
### Dockerfile
Two Dockerfile provided, for using with or without GPU. Built docker images can be found in [This DockerHub repositoriy](https://hub.docker.com/repository/docker/alienkh/ros2_rover/general). 
### Launch files
All required launchfiles and configs are placed in launchfile folder.
### Compose files
This compose files are ment to launch whole project with various configurations. The ones with jetson in their names are going to use jetson orin nx GPU in their contianers (-D CUDA_ARCH_BIN=8.7 in building opencv cmake command which is for orin nx).
```
# To start
xhost +local:docker
docker compose -f <file name> up -d 
# To stop
docker compose -f <file name> down
```
### Health Check
Health Checks are added to make user able to monitor fuctionality of each container. It's functionality can be expanded by using ELK or monitoring stacks. It is recommended to have health check for confirming each tool to make sure it is running correctly.



## Use Separately 

### [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)
Driver to use ROS2 for [SLAMTEC](https://www.slamtec.com/en) LIDAR. all instructions needed are writen in the very link. It's worth to mention that you need to publish tf yourself. It can be used in slams like gmapping, slam_toolbox and Cartographer.
To run Rtabmap change :
```
mkdir -p ws_rover/src && cd ws_rover/
git clone https://github.com/Slamtec/sllidar_ros2 ./src
source /opt/ros/$ROS_DISTRO/setup.bash && colcon buld --symlink-install
source ./install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py # remove "view" for starting without rviz
```
### using Rtabmap for odometry with just lidar
use fallowing commands for now:
```
/opt/ros/$ROS_DISTRO/share/rtabmap_demos/launch/turtlebot3/turtlebot3_scan.launch.py
'frame_id':'base_link'
localization', default_value='true'

ros2 run rtabmap_odom icp_odometry
ros2 launch rtabmap_demos turtlebot3_scan.launch.py
# or use launchfile in this repo
ros2 launch /home/rover/ws_rover/volume/launchFiles/rtabmap_icp_lidar.launch.py

```