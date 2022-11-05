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

PARAM_DIRNAME=$(READ_DATAINFO ${DATAINFO_FILE} ${PARAM_DIRNAME})
PARAM_DIRPATH=$(rospack find ${PARAM_DIRNAME})

roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/autoware/waypoint_loader.launch \
  waypoint_replanner_yamlpath:=${PARAM_DIRPATH}/autoware/waypoint_replanner.yaml \