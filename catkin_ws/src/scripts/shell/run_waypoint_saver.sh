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

roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/autoware/waypoint_saver.launch \
  save_filename:=${DATA_DIR}/${RAW_WAYPOINT_FILENAME} \
  interval:=0.1 \
  pose_topic:="/current_pose" \
  velocity_topic:="/estimated_vel_mps" \
  save_velocity:="False" \
