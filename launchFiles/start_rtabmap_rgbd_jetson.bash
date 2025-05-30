#!/bin/bash
### Run Rtabmap SLAM on jetson 
source /opt/ros/$ROS_DISTRO/setup.bash
### For RGBD cameras
ros2 launch rtabmap_launch rtabmap.launch.py   \
     args:='-d  --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 6 --Grid/MapFrameProjection true     \
     --NormalsSegmentation false --Grid/MaxGroundHeight 0.5  --Grid/MaxObstacleHeight 2.2 --RGBD/StartAtOrigin true          \
     --Grid/RayTracing true --Grid/3D true  --Grid/FlatObstacleDetected true --Grid/MaxGroundHeight 0.0                      \
     --Grid/MaxObstacleHeight "2.0"  --RGBD/MaxDepth 5.0 \
     --Optimizer/UseCUDA true   --Vis/FeatureType 6  --Optimizer/Strategy 1 --RGBD/OptimizeWithGPU true '\
     rgb_topic:=/oak/rgb/image_raw   depth_topic:=/oak/stereo/image_raw    camera_info_topic:=/oak/rgb/camera_info           \
     imu_topic:=/imu/data     wait_imu_to_init:=true     approx_sync:=true  approx_sync_max_interval:=0.01                   \
     use_sim_time:=false  qos:=2    rtabmapviz:=true     rviz:=false   subscribe_rgbd:=false

### For Stereo cameras
# ros2 launch rtabmap_launch rtabmap.launch.py \
#      args:="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10  --Kp/DetectorStrategy 10  \
#      --Grid/MapFrameProjection true  --NormalsSegmentation false --Grid/MaxGroundHeight 1.0 \
#      --Grid/MaxObstacleHeight 2.0 --RGBD/StartAtOrigin true" \
#      --Optimizer/UseCUDA true   --Vis/FeatureType 6  --Optimizer/Strategy 1 --RGBD/OptimizeWithGPU true \
#      stereo:=true  \
#      left_image_topic:=/left/image_rect    left_camera_info_topic:=/left/camera_info    \
#      right_image_topic:=/right/image_rect  right_camera_info_topic:=/right/camera_info   \
#      imu_topic:=/imu/data  frame_id:=oak-d-base-frame  \
#      approx_sync:=true  wait_imu_to_init:=true    \
#      qos:=2  rtabmapviz:=true  rviz:=false     # approx_sync_max_interval:=0.001 

