#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# X. Introduce environment variables.
source ${SCRIPT_DIR}/build_env.sh

echo ${glog_DIR}

catkin_make \
  -Dglog_DIR=${glog_DIR} \
	--force-cmake
