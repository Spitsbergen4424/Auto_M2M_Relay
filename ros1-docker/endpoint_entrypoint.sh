#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

exec roslaunch ros_tcp_endpoint endpoint.launch \
  tcp_ip:=0.0.0.0 \
  tcp_port:=10000
