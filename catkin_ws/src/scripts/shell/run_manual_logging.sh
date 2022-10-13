#!/bin/bash

if [ $# -eq 1 ]; then
  if [ ${1} -eq 0 ]; then
    USE_GPS=False
  else
    USE_GPS=True
    ${ROS_SCRIPTS_PKG}/shell/run_rtklib_str2str.sh &
    sleep 5
  fi
else
  echo "Please specify argument." 1>&2
  exit 1
fi

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh

YPSPUR_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/yp_spur.yaml
CTRL_LIMIT_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/control_limit.yaml
YPSPUR_BIN_FILEPATH=${ROS_SCRIPTS_PKG}/../../../../3rdparty/install/yp_spur/bin/ypspur-coordinator
YPSPUR_PARAM_PATH=${ROS_SCRIPTS_PKG}/params/common/yp_spur.param
URG_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/urg_node.yaml
VELODYNE_CALIBFILE="$(rospack find velodyne_pointcloud)/params/VLP16db.yaml"
VELODYNE_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/velodyne.yaml
RVIZ_CONF=${ROS_SCRIPTS_PKG}/rviz/config.rviz
URDF_FILEPATH=${ROS_SCRIPTS_PKG}/urdf/robot.urdf
ROBOT_FILEPATH=${ROS_SCRIPTS_PKG}/params/common/robot.yaml
ODOM_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/odometry.yaml
IMU_ADJUST_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/imu_adjust.yaml
NMEA_NAVSAT_DRIVER_YAMLPATH=${ROS_SCRIPTS_PKG}/params/common/nmea_navsat_driver.yaml

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/manual_logging.launch \
  base_frame_id:="base_link" \
  imu_frame_id:="imu_link" \
  urg_frame_id:="laser2d_link" \
  velodyne_frame_id:="laser3d_link" \
  ypspur_odom_frame_id:="odom_ypspur" \
  ypspur_cmd_vel:="cmd_vel" \
  yp_spur_yaml_filepath:=${YPSPUR_YAMLPATH} \
  control_limit_filepath:=${CTRL_LIMIT_YAMLPATH} \
  ypspur_bin_filepath:=${YPSPUR_BIN_FILEPATH} \
  ypspur_param_filepath:=${YPSPUR_PARAM_PATH} \
  imu_port:="/dev/RT_USB_IMU" \
  urg_node_yaml_filepath:=${URG_YAMLPATH} \
  velodyne_calibration:=${VELODYNE_CALIBFILE} \
  velodyne_param_filepath:=${VELODYNE_YAMLPATH} \
  rviz_config_file:=${RVIZ_CONF} \
  urdf_file:=${URDF_FILEPATH} \
  odom_frame_id:="odom" \
  robot_yaml_filepath:=${ROBOT_FILEPATH} \
  odom_yaml_filepath:=${ODOM_YAMLPATH} \
  imu_adjust_param_filepath:=${IMU_ADJUST_YAMLPATH} \
  use_gps:=${USE_GPS} \
	nmea_driver_param_filepath:=${NMEA_NAVSAT_DRIVER_YAMLPATH} \
