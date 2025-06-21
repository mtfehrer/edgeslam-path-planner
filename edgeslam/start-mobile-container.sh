#!/bin/bash
source /opt/ros/melodic/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/edgeslam/Examples/ROS
roscore &
cd /home/edgeslam/Examples/ROS/Edge_SLAM/
rosrun Edge_SLAM RGBD ../../../Vocabulary/ORBvoc.txt ../../RGB-D/TUM2.yaml client < /home/edgeslam/mobile-input.txt &
sleep 20
rosbag play ./rgbd_dataset_freiburg2_desk.bag /camera/rgb/image_color:=/camera/rgb/image_raw /camera/depth/image:=/camera/depth_registered/image_raw
