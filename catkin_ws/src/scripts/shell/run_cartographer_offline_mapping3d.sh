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

CONFIG_DIR_PATH="${PARAM_DIRPATH}/cartographer/configuration_files"

PG_FILEPATH=${BAG_FILEPATH/".bag"/".bag.pbstream"}
PCD_FILEPATH=${BAG_FILEPATH/".bag"/".bag_points.pcd"}
FILTERED_PCD_FILEPATH=${BAG_FILEPATH/".bag"/".bag_points_flt.pcd"}
CONFIG_BASENAME="mapping_3d.lua"

roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/cartographer/offline_mapping_3d.launch \
	config_dirpath:=${CONFIG_DIR_PATH} \
  config_basename:=${CONFIG_BASENAME} \
  urdf_filepath:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  points2_name:="/velodyne_points" \
  bag_filepath:=${BAG_FILEPATH}

roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/cartographer/assets_writer_3d.launch \
  config_dirpath:=${CONFIG_DIR_PATH} \
  urdf_filepath:=${ROS_SCRIPTS_PKG}/urdf/robot.urdf \
  bag_filenames:=${BAG_FILEPATH} \
  pose_graph_filename:=${PG_FILEPATH}

pcl_voxel_grid ${PCD_FILEPATH} ${FILTERED_PCD_FILEPATH} -leaf 0.5,0.5,0.5
