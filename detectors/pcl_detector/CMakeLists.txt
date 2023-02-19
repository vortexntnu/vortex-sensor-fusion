cmake_minimum_required(VERSION 3.0.2)
project(pcl_detector)

set(BINARY_NAME ${PROJECT_NAME}_ros_node)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
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
)

target_link_libraries(${BINARY_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
