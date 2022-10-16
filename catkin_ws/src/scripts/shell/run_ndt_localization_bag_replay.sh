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

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/ndt_localization_bag_replay.launch \
  urdf_file:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  plane_number:=$(READ_DATAINFO ${DATAINFO_FILE} ${PLANE_NUMBER}) \
  robot_yaml_filepath:=${PARAM_DIRPATH}/common/robot.yaml \
  odom_yaml_filepath:=${PARAM_DIRPATH}/common/odometry.yaml \
  imu_adjust_param_filepath:=${PARAM_DIRPATH}/common/imu_adjust.yaml \
  pcd_filelist:=${MAP_PCDFILE} \
  raw_points_topic:="/velodyne_points" \
  downsampler_node_name:="voxel_grid_filter" \
  common_yaml_filepath:=${PARAM_DIRPATH}/autoware/common.yaml \
  ndt_matching_yaml_filepath:=${PARAM_DIRPATH}/autoware/ndt_matching.yaml \
  mcl_3dl_yamlpath:=${PARAM_DIRPATH}/common/mcl_3dl.yaml \
  pose_initializer:=$(READ_DATAINFO ${DATAINFO_FILE} ${POSE_INITIALIZER}) \
  init_via_gnss:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_VIA_GNSS}) \
  x:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_X_NAME}) \
  y:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_Y_NAME}) \
  z:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_Z_NAME}) \
  roll:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_ROLL_NAME}) \
  pitch:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_PITCH_NAME}) \
  yaw:=$(READ_DATAINFO ${DATAINFO_FILE} ${INIT_YAW_NAME}) \
  bagfile_path:=${BAG_FILEPATH} \
  start:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_START}) \
  duration:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_DURATION}) \
  rate:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_RATE}) \
  wait:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_WAIT}) \
  dump_pose_file:="False" \
  localized_pose_filepath:=${DATA_DIR}/${LOCALIZED_POSE_JSON} \
  pose_pair_filepath:=${DATA_DIR}/${POSE_PAIR_JSON} \
  kml_filepath:=${DATA_DIR}/${KML_FILENAME} \
  world_to_map_json:=${DATA_DIR}/${WORLD_TO_MAP_JSON} \
  rviz_config_file:=${PARAM_DIRPATH}/rviz/localization.rviz \
