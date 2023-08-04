cmake_minimum_required(VERSION 3.0.2)
project(sift_feature_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

catkin_package(
)

catkin_install_python(PROGRAMS
  scripts/sift_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

