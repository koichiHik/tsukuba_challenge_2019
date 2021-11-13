#!/bin/bash

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh
source ${ROS_SCRIPTS_PKG}/shell/ros_general_functions.sh

#ROSBAG_RECORD_BASE_TOPICS_AND_ODOM
ROSBAG_RECORD_RAW_TOPICS
