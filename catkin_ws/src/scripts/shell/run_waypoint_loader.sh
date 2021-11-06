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

WAYPOINT_REPLANNER_YAMLPATH=${ROS_SCRIPTS_PKG}/params/autoware/waypoint_replanner.yaml

roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/autoware/waypoint_loader.launch \
  multi_lane_csv:=${DATA_DIR}/${WAYPOINT_FILENAME} \
  waypoint_replanner_yamlpath:=${WAYPOINT_REPLANNER_YAMLPATH} \
