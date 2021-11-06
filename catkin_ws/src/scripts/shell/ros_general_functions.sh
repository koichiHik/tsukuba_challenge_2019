#!/bin/bash

function CREATE_LOG_DIR() {

  DIR_NAME=${1}
  DATA_DIR="${HOME}/LOG/$(date '+%Y%m%d')"
  mkdir -p ${DATA_DIR}

  NEW_DIR=${DATA_DIR}/${DIR_NAME}
  if [ ! -d ${NEW_DIR} ]; then
    mkdir ${NEW_DIR}
  fi

  echo "${NEW_DIR}"
}

function GET_BAGFILE_PATH() {
  local BAG_DIR=${1}
  local BAGFILE=$(find ${BAG_DIR} -maxdepth 1 -type f -name "*.bag")
  echo ${BAGFILE}
}

function GET_PCDFILE_PATH() {
  local PCD_DIR=${1}
  local PCDFILE=$(find ${PCD_DIR} -maxdepth 1 -type f -name "*.pcd")
  echo "[${PCDFILE}]"
}

function ROSBAG_EXTRACT_RAW_TOPICS() {

  local SRCBAG=${1}
	local NEWBAG=${SRCBAG/".bag"/"_raw.bag"}

  local TOPIC_NAMES="topic == '/imu_raw'" 
  TOPIC_NAMES+=" or topic == '/imu/mag'" 
  TOPIC_NAMES+=" or topic == '/imu/temperature'"
  TOPIC_NAMES+=" or topic == '/scan'"
  TOPIC_NAMES+=" or topic == '/velodyne/scan'"
  TOPIC_NAMES+=" or topic == '/velodyne_packets'"
  TOPIC_NAMES+=" or topic == '/velodyne_points'"
  TOPIC_NAMES+=" or topic == '/ypspur/control_mode'"
  TOPIC_NAMES+=" or topic == '/ypspur/diagnostics'"
  TOPIC_NAMES+=" or topic == '/ypspur/joint_states'"
  TOPIC_NAMES+=" or topic == '/ypspur/wrench'"
	TOPIC_NAMES+=" or topic == '/fix'"
	TOPIC_NAMES+=" or topic == '/time_reference'"
	TOPIC_NAMES+=" or topic == '/vel'"

	rosbag filter ${SRCBAG} ${NEWBAG} "${TOPIC_NAMES}"
}

function ROSBAG_RECORD_RAW_TOPICS() {

  BAG_RECORD_DIR=$(CREATE_LOG_DIR rosbag)/rec

  local TOPIC_NAMES="/imu_raw "
  TOPIC_NAMES+="/imu/mag " 
  TOPIC_NAMES+="/imu/temperature "
  TOPIC_NAMES+="/scan "
  TOPIC_NAMES+="/velodyne/scan "
  TOPIC_NAMES+="/velodyne_packets "
  TOPIC_NAMES+="/velodyne_points "
  TOPIC_NAMES+="/ypspur/control_mode "
  TOPIC_NAMES+="/ypspur/diagnostics "
  TOPIC_NAMES+="/ypspur/joint_states "
  TOPIC_NAMES+="/ypspur/wrench"
	TOPIC_NAMES+="/fix "
	TOPIC_NAMES+="/time_reference "
	TOPIC_NAMES+="/vel "

  rosbag record -o ${BAG_RECORD_DIR} ${TOPIC_NAMES}

}

function ROSBAG_RECORD_BASE_TOPICS_AND_ODOM() {

  BAG_RECORD_DIR=$(CREATE_LOG_DIR rosbag)/rec

  local TOPIC_NAMES="/imu_raw "
  TOPIC_NAMES+="/imu/mag "
  TOPIC_NAMES+="/imu/temperature "
  TOPIC_NAMES+="/scan "
  TOPIC_NAMES+="/velodyne/scan "
  TOPIC_NAMES+="/velodyne_packets "
  TOPIC_NAMES+="/velodyne_points "
  TOPIC_NAMES+="/ypspur/control_mode "
  TOPIC_NAMES+="/ypspur/diagnostics "
  TOPIC_NAMES+="/ypspur/joint_states "
  TOPIC_NAMES+="/ypspur/wrench "
	TOPIC_NAMES+="/fix "
	TOPIC_NAMES+="/time_reference "
	TOPIC_NAMES+="/vel "
  TOPIC_NAMES+="/odom "
  TOPIC_NAMES+="/imu "

  rosbag record -o ${BAG_RECORD_DIR} ${TOPIC_NAMES}

}
