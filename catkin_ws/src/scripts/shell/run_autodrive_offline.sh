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
DATAINFO_FILE=${DATA_DIR}/${DATAINFO_FILENAME}
BAG_FILEPATH=$(GET_BAGFILE_PATH ${DATA_DIR})
MAP_PCDFILE=$(GET_PCDFILE_PATH ${DATA_DIR})

PARAM_DIRNAME=$(READ_DATAINFO ${DATAINFO_FILE} ${PARAM_DIRNAME})
PARAM_DIRPATH=$(rospack find ${PARAM_DIRNAME})

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/autodrive_offline.launch \
  urdf_file:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  map_frame_id:="map" \
  lidar_frame_id:="laser3d_link" \
  robot_yaml_filepath:=${PARAM_DIRPATH}/common/robot.yaml \
  odom_yaml_filepath:=${PARAM_DIRPATH}/common/odometry.yaml \
  imu_adjust_param_filepath:=${PARAM_DIRPATH}/common/imu_adjust.yaml \
  rviz_config_file:=${PARAM_DIRPATH}/rviz/config.rviz \
  plane_number:=$(READ_DATAINFO ${DATAINFO_FILE} ${PLANE_NUMBER}) \
  pcd_filelist:=${MAP_PCDFILE} \
  raw_points_topic:="/velodyne_points" \
  downsampler_node_name:="voxel_grid_filter" \
  common_yaml_filepath:=${PARAM_DIRPATH}/autoware/common.yaml \
  ndt_matching_yaml_filepath:=${PARAM_DIRPATH}/autoware/ndt_matching.yaml \
  mcl_3dl_yamlpath:=${PARAM_DIRPATH}/common/mcl_3dl.yaml \
  world_to_map_json:=${DATA_DIR}/${WORLD_TO_MAP_JSON} \
  pose_initializer:=$(READ_DATAINFO ${DATAINFO_FILE} ${POSE_INITIALIZER}) \
  init_via_gnss:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_VIA_GNSS}) \
  x:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_X_NAME}) \
  y:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_Y_NAME}) \
  z:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_Z_NAME}) \
  roll:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_ROLL_NAME}) \
  pitch:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_PITCH_NAME}) \
  yaw:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_YAW_NAME}) \
  lidar_cluster_filepath:=${PARAM_DIRPATH}/autoware/lidar_euclidean_cluster_detect.yaml \
  costmap_generator_filepath:=${PARAM_DIRPATH}/autoware/costmap_generator.yaml \
  astar_avoid_yamlpath:=${PARAM_DIRPATH}/autoware/astar_avoid.yaml \
  velocity_set_yamlpath:=${PARAM_DIRPATH}/autoware/velocity_set.yaml \
  pure_pursuit_controller_yamlpath:=${PARAM_DIRPATH}/autoware/pure_pursuit_controller.yaml \
  lane_navi_yamlpath:=${PARAM_DIRPATH}/autoware/lane_navi.yaml \
  lane_rule_yamlpath:=${PARAM_DIRPATH}/autoware/lane_rule.yaml \
  lane_select_yamlpath:=${PARAM_DIRPATH}/autoware/lane_select.yaml \
  bagfile_path:=${BAG_FILEPATH} \
  start:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_START}) \
  duration:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_DURATION}) \
  rate:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_RATE}) \
  wait:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_WAIT}) \
  status_management_yamlpath:=${PARAM_DIRPATH}/common/status_management.yaml
