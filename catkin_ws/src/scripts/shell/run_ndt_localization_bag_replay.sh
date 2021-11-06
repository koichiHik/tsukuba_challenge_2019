#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify bag filepath."
  exit 1
fi

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh
source ${ROS_SCRIPTS_PKG}/shell/ros_general_functions.sh
source ${ROS_SCRIPTS_PKG}/shell/filename_solve.sh

DATA_DIR=${1}
PLANE_NUMBER="4"
BAG_FILEPATH=$(GET_BAGFILE_PATH ${DATA_DIR})
MAP_PCDFILE=$(GET_PCDFILE_PATH ${DATA_DIR})

URDF_FILEPATH=${ROS_SCRIPTS_PKG}/urdf/robot.urdf
ROBOT_YAML_FILEPATH=${ROS_SCRIPTS_PKG}/params/common/robot.yaml
ODOM_YAML_FILEPATH=${ROS_SCRIPTS_PKG}/params/common/odometry.yaml
IMU_ADJUST_FILEPATH=${ROS_SCRIPTS_PKG}/params/common/imu_adjust.yaml
AT_COMMON_YAML_FILEPATH=${ROS_SCRIPTS_PKG}/params/autoware/common.yaml
AT_NDT_MATCHING_FILEPATH=${ROS_SCRIPTS_PKG}/params/autoware/ndt_matching.yaml

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/ndt_localization_bag_replay.launch \
  urdf_file:=${URDF_FILEPATH} \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  plane_number:=${PLANE_NUMBER} \
  robot_yaml_filepath:=${ROBOT_YAML_FILEPATH} \
  odom_yaml_filepath:=${ODOM_YAML_FILEPATH} \
  imu_adjust_param_filepath:=${IMU_ADJUST_FILEPATH} \
  pcd_filelist:=${MAP_PCDFILE} \
  raw_points_topic:="/velodyne_points" \
  downsampler_node_name:="voxel_grid_filter" \
  common_yaml_filepath:=${AT_COMMON_YAML_FILEPATH} \
  ndt_matching_yaml_filepath:=${AT_NDT_MATCHING_FILEPATH} \
  x:=4.0 \
  y:=2.0 \
  z:=0.0 \
  roll:=0.0 \
  yaw:=90.0 \
  pitch:=0.0 \
  bagfile_path:=${BAG_FILEPATH} \
  start:=50.0 \
  duration:=0.0 \
  rate:=1.0 \
  wait:=7.0 \
  dump_pose_file:="False" \
  localized_pose_filepath:=${DATA_DIR}/${LOCALIZED_POSE_JSON} \
  pose_pair_filepath:=${DATA_DIR}/${POSE_PAIR_JSON} \
  kml_filepath:=${DATA_DIR}/${KML_FILENAME} \
  world_to_map_json:=${DATA_DIR}/${WORLD_TO_MAP_JSON} \
  rviz_config_file:=${ROS_SCRIPTS_PKG}/rviz/localization.rviz \
