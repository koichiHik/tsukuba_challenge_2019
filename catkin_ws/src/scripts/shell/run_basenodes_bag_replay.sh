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

PARAM_DIRNAME=$(READ_DATAINFO ${DATAINFO_FILE} ${PARAM_DIRNAME})
PARAM_DIRPATH=$(rospack find ${PARAM_DIRNAME})
echo ${PARAM_DIRPATH}

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/basenodes_bag_replay.launch \
  urdf_file:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  robot_yaml_filepath:=${PARAM_DIRPATH}/common/robot.yaml \
  odom_yaml_filepath:=${PARAM_DIRPATH}/common/odometry.yaml \
  imu_adjust_param_filepath:=${PARAM_DIRPATH}/common/imu_adjust.yaml \
  rviz_config_file:=${PARAM_DIRPATH}/rviz/config.rviz \
  bagfile_path:=${BAG_FILEPATH} \
  start:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_START}) \
  duration:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_DURATION}) \
  rate:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_RATE}) \
  wait:=$(READ_DATAINFO ${DATAINFO_FILE} ${BAG_WAIT}) \
