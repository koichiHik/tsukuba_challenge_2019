#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify bag filepath."
  exit 1
fi

START_COURSE_IDX=0

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh
source ${ROS_SCRIPTS_PKG}/shell/ros_general_functions.sh
source ${ROS_SCRIPTS_PKG}/shell/filename_solve.sh

DATA_DIR=${1}
DATAINFO_FILE=${DATA_DIR}/${DATAINFO_FILENAME}

PARAM_DIRNAME=$(READ_DATAINFO ${DATAINFO_FILE} ${PARAM_DIRNAME})
PARAM_DIRPATH=$(rospack find ${PARAM_DIRNAME})

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/autodrive_base.launch \
  urdf_file:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  base_frame_id:="base_link" \
  odom_frame_id:="odom" \
  map_frame_id:="map" \
  lidar_frame_id:="laser3d_link" \
  yp_spur_yaml_filepath:=${PARAM_DIRPATH}/common/yp_spur.yaml \
  control_limit_filepath:=${PARAM_DIRPATH}/common/control_limit.yaml \
  ypspur_param_filepath:=${PARAM_DIRPATH}/common/yp_spur.param \
  urg_node_yaml_filepath:=${PARAM_DIRPATH}/common/urg_node.yaml \
  velodyne_param_filepath:=${PARAM_DIRPATH}/common/velodyne.yaml \
  robot_yaml_filepath:=${PARAM_DIRPATH}/common/robot.yaml \
  odom_yaml_filepath:=${PARAM_DIRPATH}/common/odometry.yaml \
  imu_adjust_param_filepath:=${PARAM_DIRPATH}/common/imu_adjust.yaml \
  use_gps:=True \
  nmea_driver_param_filepath:=${PARAM_DIRPATH}/common/nmea_navsat_driver.yaml \
  rviz_config_file:=${PARAM_DIRPATH}/rviz/config.rviz \
  common_yaml_filepath:=${PARAM_DIRPATH}/autoware/common.yaml \
