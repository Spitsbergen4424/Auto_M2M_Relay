cmake_minimum_required(VERSION 3.0.2)
project(gfsx_unity_bridge)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy std_msgs ros_tcp_endpoint)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/gfsx_ros1_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
