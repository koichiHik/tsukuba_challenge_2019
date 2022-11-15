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

# USE_GPS=$(READ_DATAINFO ${DATAINFO_FILE} ${USE_GPS})
# if [[ "${USE_GPS}" == "True" ]]; then
#  ${ROS_SCRIPTS_PKG}/shell/run_rtklib_str2str.sh &
#  sleep 5
# fi

PARAM_DIRNAME=$(READ_DATAINFO ${DATAINFO_FILE} ${PARAM_DIRNAME})
PARAM_DIRPATH=$(rospack find ${PARAM_DIRNAME})

YPSPUR_BIN_FILEPATH=${ROS_SCRIPTS_PKG}/../../../../3rdparty/install/yp_spur/bin/ypspur-coordinator
VELODYNE_CALIBFILE="$(rospack find velodyne_pointcloud)/params/VLP16db.yaml"

roslaunch ${ROS_SCRIPTS_PKG}/launch/system/controller_logging.launch \
  base_frame_id:="base_link" \
  imu_frame_id:="imu_link" \
  urg_frame_id:="laser2d_link" \
  velodyne_frame_id:="laser3d_link" \
  ypspur_odom_frame_id:="odom_ypspur" \
  ypspur_cmd_vel:="cmd_vel" \
  yp_spur_yaml_filepath:=${PARAM_DIRPATH}/common/yp_spur.yaml \
  control_limit_filepath:=${PARAM_DIRPATH}/common/control_limit.yaml \
  ypspur_bin_filepath:=${YPSPUR_BIN_FILEPATH} \
  ypspur_param_filepath:=${PARAM_DIRPATH}/common/yp_spur.param \
  imu_port:="/dev/RT_USB_IMU" \
  urg_node_yaml_filepath:=${PARAM_DIRPATH}/common/urg_node.yaml \
  velodyne_calibration:=${VELODYNE_CALIBFILE} \
  velodyne_param_filepath:=${PARAM_DIRPATH}/common/velodyne.yaml \
  joystick_param_filepath:=${PARAM_DIRPATH}/common/ps3_holonomic_config.yaml \
  rviz_config_file:=${PARAM_DIRPATH}/rviz/config.rviz \
  urdf_file:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  odom_frame_id:="odom" \
  robot_yaml_filepath:=${PARAM_DIRPATH}/common/robot.yaml \
  odom_yaml_filepath:=${PARAM_DIRPATH}/common/odometry.yaml \
  imu_adjust_param_filepath:=${PARAM_DIRPATH}/common/imu_adjust.yaml \
  use_gps:=True \
  nmea_driver_param_filepath:=${PARAM_DIRPATH}/common/nmea_navsat_driver.yaml \
