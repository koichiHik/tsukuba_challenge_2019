#!/bin/bash

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh


roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/autoware/ndt_localization.launch \
  pcd_filelist:="/home/koichi/Desktop/tc_data/tc21/20211009/odom_calculated/2021-10-09-14-23-10_odom.bag_points.pcd" \
  raw_points_topic:="/velodyne_points" \
  downsampler_node_name:="voxel_grid_filter" \
  common_yaml_filepath:=${ROS_SCRIPTS_PKG}/params/autoware/common.yaml \
  ndt_matching_yaml_filepath:=${ROS_SCRIPTS_PKG}/params/autoware/ndt_matching.yaml
