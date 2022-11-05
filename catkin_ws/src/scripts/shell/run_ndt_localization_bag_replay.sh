#!/bin/bash

if [ $# -ne 2 ]; then
  echo "Please specify bag filepath."
  exit 1
fi

START_COURSE_IDX=0

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh
source ${ROS_SCRIPTS_PKG}/shell/ros_general_functions.sh
source ${ROS_SCRIPTS_PKG}/shell/filename_solve.sh

DATA_DIR=${1}
COURSE_CONFIG_YAML=${2}
DATAINFO_FILE=${DATA_DIR}/${DATAINFO_FILENAME}
BAG_FILEPATH=$(GET_BAGFILE_PATH ${DATA_DIR})
MAP_PCDFILE=$(GET_PCDFILE_PATH ${DATA_DIR})

PARAM_DIRNAME=$(READ_DATAINFO ${DATAINFO_FILE} ${PARAM_DIRNAME})
PARAM_DIRPATH=$(rospack find ${PARAM_DIRNAME})

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/ndt_localization_bag_replay.launch \
  urdf_file:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  robot_yaml_filepath:=${PARAM_DIRPATH}/common/robot.yaml \
  odom_yaml_filepath:=${PARAM_DIRPATH}/common/odometry.yaml \
  imu_adjust_param_filepath:=${PARAM_DIRPATH}/common/imu_adjust.yaml \
  raw_points_topic:="/velodyne_points" \
  downsampler_node_name:="voxel_grid_filter" \
  common_yaml_filepath:=${PARAM_DIRPATH}/autoware/common.yaml \
  ndt_matching_yaml_filepath:=${PARAM_DIRPATH}/autoware/ndt_matching.yaml \
  mcl_3dl_yamlpath:=${PARAM_DIRPATH}/common/mcl_3dl.yaml \
  bagfile_path:=${BAG_FILEPATH} \
  start:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_START}) \
  duration:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_DURATION}) \
  rate:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_RATE}) \
  wait:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_WAIT}) \
  dump_pose_file:="False" \
  localized_pose_filepath:=${DATA_DIR}/${LOCALIZED_POSE_JSON} \
  pose_pair_filepath:=${DATA_DIR}/${POSE_PAIR_JSON} \
  kml_filepath:=${DATA_DIR}/${KML_FILENAME} \
  rviz_config_file:=${PARAM_DIRPATH}/rviz/localization.rviz \
  start_course_idx:=${START_COURSE_IDX} \
  course_config_yamlpath:=${COURSE_CONFIG_YAML} \
  status_management_yamlpath:=${PARAM_DIRPATH}/common/status_management.yaml \
