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
PLANE_NUMBER="4" # Ibaraki : 9, Tokushima : 4
BAG_FILEPATH=$(GET_BAGFILE_PATH ${DATA_DIR})
MAP_PCDFILE=$(GET_PCDFILE_PATH ${DATA_DIR})

URDF_FILEPATH=${ROS_SCRIPTS_PKG}/urdf/robot.urdf
ROBOT_FILEPATH=${ROS_SCRIPTS_PKG}/params/common/robot.yaml
ODOM_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/odometry.yaml
IMU_ADJUST_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/imu_adjust.yaml
AT_COMMON_YAML_FILEPATH=${ROS_SCRIPTS_PKG}/params/autoware/common.yaml
AT_NDT_MATCHING_FILEPATH=${ROS_SCRIPTS_PKG}/params/autoware/ndt_matching.yaml
LIDAR_CLUSTER_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/lidar_euclidean_cluster_detect.yaml
COSTMAP_GENERATOR_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/costmap_generator.yaml
ASTAR_AVOID_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/astar_avoid.yaml
VELOCITY_SET_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/velocity_set.yaml
PURE_PURSUIT_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/pure_pursuit_controller.yaml
LANE_NAVI_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/lane_navi.yaml
LANE_RULE_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/lane_rule.yaml
LANE_SELECT_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/lane_select.yaml
RVIZ_CONF=${ROS_SCRIPTS_PKG}/rviz/config.rviz

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/autodrive_offline.launch \
  urdf_file:=${URDF_FILEPATH} \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  map_frame_id:="map" \
  lidar_frame_id:="laser3d_link" \
  robot_yaml_filepath:=${ROBOT_FILEPATH} \
  odom_yaml_filepath:=${ODOM_YAMLPATH} \
  imu_adjust_param_filepath:=${IMU_ADJUST_YAMLPATH} \
  rviz_config_file:=${RVIZ_CONF} \
  plane_number:=${PLANE_NUMBER} \
  pcd_filelist:=${MAP_PCDFILE} \
  raw_points_topic:="/velodyne_points" \
  downsampler_node_name:="voxel_grid_filter" \
  common_yaml_filepath:=${AT_COMMON_YAML_FILEPATH} \
  ndt_matching_yaml_filepath:=${AT_NDT_MATCHING_FILEPATH} \
  world_to_map_json:=${DATA_DIR}/${WORLD_TO_MAP_JSON} \
  x:=0.0 \
  y:=0.0 \
  z:=0.0 \
  roll:=0.0 \
  yaw:=0.0 \
  pitch:=0.0 \
  lidar_cluster_filepath:=${LIDAR_CLUSTER_YAMLPATH} \
  costmap_generator_filepath:=${COSTMAP_GENERATOR_YAMLPATH} \
  astar_avoid_yamlpath:=${ASTAR_AVOID_YAMLPATH} \
  velocity_set_yamlpath:=${VELOCITY_SET_YAMLPATH} \
  pure_pursuit_controller_yamlpath:=${PURE_PURSUIT_YAMLPATH} \
  lane_navi_yamlpath:=${LANE_NAVI_YAMLPATH} \
  lane_rule_yamlpath:=${LANE_RULE_YAMLPATH} \
  lane_select_yamlpath:=${LANE_SELECT_YAMLPATH} \
  bagfile_path:=${BAG_FILEPATH} \
  start:=0.0 \
  duration:=0.0 \
  rate:=1.0 \
  wait:=3
