cmake_minimum_required(VERSION 3.0.2)
project(colmag_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/colmag_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
