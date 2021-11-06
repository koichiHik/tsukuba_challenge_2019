#!/bin/bash

ROS_SCRIPTS_PKG=$(rospack find scripts)
source ${ROS_SCRIPTS_PKG}/shell/run_env.sh
source ${ROS_SCRIPTS_PKG}/shell/env_variable_solve.sh

function cleanup {
  kill ${PID1}
  kill ${PID2}
  exit
}
trap cleanup SIGHUP SIGINT SIGTERM

socat -d -d pty,link=/home/koichi/FAKE_UBLOX_IN,raw,echo=0 \
            pty,link=/home/koichi/FAKE_UBLOX_OUT,raw,echo=0 &
PID1=$!
sleep 5

${rtklib_BIN_DIR}/str2str \
  -in serial://UBLOX_F9P_USB:230400:8:n:1 \
  -out ntrip://${ICHIMILL_ID}:${ICHIMILL_PASS}@${ICHIMILL_URL}:${ICHIMILL_PORT}/${ICHIMILL_MPNT} \
  -b 1 \
  -out serial:://../home/koichi/FAKE_UBLOX_IN:230400:8:n:1 &
PID2=$!

wait
