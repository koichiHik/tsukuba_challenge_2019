#!/bin/bash

# X. Library directory top
LIB_DIR_TOP="/home/koichi/workspace/3rdparty/install"

# X. Path to dependency
gflags_LIB_DIR="${LIB_DIR_TOP}/gflags/lib"
glog_LIB_DIR="${LIB_DIR_TOP}/glog/lib"
rtklib_BIN_DIR="${LIB_DIR_TOP}/rtklib/bin"
ypspur_LIB_DIR="${LIB_DIR_TOP}/yp_spur/lib"

LD_LIBRARY_PATH=${gflags_LIB_DIR}:${glog_LIB_DIR}:${LD_LIBRARY_PATH}
LD_LIBRARY_PATH=${ypspur_LIB_DIR}:${LD_LIBRARY_PATH}