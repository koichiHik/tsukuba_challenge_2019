#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify bag filepath."
  exit 1
fi

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh

BAG_PATH=${1}
URDF_FILEPATH=${ROS_SCRIPTS_PKG}/urdf/robot.urdf
ROBOT_FILEPATH=${ROS_SCRIPTS_PKG}/params/common/robot.yaml
ODOM_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/odometry.yaml
IMU_ADJUST_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/imu_adjust.yaml
RVIZ_CONF=${ROS_SCRIPTS_PKG}/rviz/config.rviz
PLANE_NUMBER="9" # Ibaraki : 9, Tokushima : 4

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/basenodes_bag_replay.launch \
  bagfile_path:=${BAG_PATH} \
  start:=0.0 \
  duration:=0.0 \
  rate:=1.0 \
  wait:=2.0 \
  urdf_file:=${URDF_FILEPATH} \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  plane_number:=${PLANE_NUMBER} \
  robot_yaml_filepath:=${ROBOT_FILEPATH} \
  odom_yaml_filepath:=${ODOM_YAMLPATH} \
  imu_adjust_param_filepath:=${IMU_ADJUST_YAMLPATH} \
  rviz_config_file:=${RVIZ_CONF}