cmake_minimum_required(VERSION 3.0.2)
project(colmag_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/colmag_node.py
  scripts/colmag_sensor_node.py
  scripts/colmag_classifier_node.py
  scripts/colmag_joystick_node.py
  scripts/colmag_robot_node.py
  scripts/colmag_draw_node.py
  scripts/fr3_simple_move.py
  scripts/fr3_recover.py
  scripts/colmag_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
