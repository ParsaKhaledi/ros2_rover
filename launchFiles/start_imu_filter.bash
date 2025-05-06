#!/bin/bash
sudo sed -i 's/use_mag: true/use_mag: false/g' $(ros2 pkg prefix imu_filter_madgwick)/share/imu_filter_madgwick/config/imu_filter.yaml && \
sudo sed -i 's/publish_tf: true/publish_tf: false/g' $(ros2 pkg prefix imu_filter_madgwick)/share/imu_filter_madgwick/config/imu_filter.yaml && \
sudo sed -i 's/world_frame: "enu"/world_frame: "map"/g' $(ros2 pkg prefix imu_filter_madgwick)/share/imu_filter_madgwick/config/imu_filter.yaml 
ros2 launch imu_filter_madgwick imu_filter_component.launch.py