#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify bag filepath."
  exit 1
fi

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh

BAG_PATH=${1}
PG_FILEPATH=${BAG_PATH/".bag"/".bag.pbstream"}
PCD_FILEPATH=${BAG_PATH/".bag"/".bag_points.pcd"}
FILTERED_PCD_FILEPATH=${BAG_PATH/".bag"/".bag_points_flt.pcd"}
ROS_SCRIPTS_PKG=$(rospack find scripts)
#CONFIG_DIR_PATH="${ROS_SCRIPTS_PKG}/params/cartographer/configuration_files"
CONFIG_DIR_PATH="/home/koichi/Desktop/autorun_nuv/params/configuration_files"
CONFIG_BASENAME="mapping_3d.lua"
URDF_FILEPATH="${ROS_SCRIPTS_PKG}/urdf/robot.urdf"

roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/cartographer/offline_mapping_3d.launch \
	config_dirpath:=${CONFIG_DIR_PATH} \
  config_basename:=${CONFIG_BASENAME} \
  urdf_filepath:=${URDF_FILEPATH} \
  points2_name:="/velodyne_points" \
  bag_filepath:=${BAG_PATH}

roslaunch ${ROS_SCRIPTS_PKG}/launch/modules/cartographer/assets_writer_3d.launch \
  config_dirpath:=${CONFIG_DIR_PATH} \
  urdf_filepath:=${URDF_FILEPATH} \
  bag_filenames:=${BAG_PATH} \
  pose_graph_filename:=${PG_FILEPATH}

pcl_voxel_grid ${PCD_FILEPATH} ${FILTERED_PCD_FILEPATH} -leaf 0.5,0.5,0.5
