#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# X. Library directory top.
LIB_DIR_TOP="${SCRIPT_DIR}/../../3rdparty/install"

# X. 3rd party library.
glog_DIR="${LIB_DIR_TOP}/glog/lib/cmake/glog"