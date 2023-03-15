cmake_minimum_required(VERSION 3.0.2)
project(pcl_detector)

set(BINARY_NAME ${PROJECT_NAME}_ros_node)
set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    pcl_ros
    dynamic_reconfigure
)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Detector.cfg
)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(${BINARY_NAME}
    src/pcl_detector_ros_node.cpp
    src/pcl_detector_ros.cpp
    src/euclidean_clustering.cpp
    src/dbscan_detector.cpp
    src/optics_detector.cpp
    src/gmm_detector.cpp
)

target_link_libraries(${BINARY_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
