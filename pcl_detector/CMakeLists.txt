cmake_minimum_required(VERSION 3.5)
project(pcl_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common)
find_package(PCL REQUIRED)

include_directories(include)


add_executable(${PROJECT_NAME}_node 
  src/pcl_detector_ros_node.cpp 
  src/pcl_detector_ros.cpp 
  src/euclidean_clustering.cpp
  src/dbscan_detector.cpp
)

target_link_libraries( ${PROJECT_NAME}_node
  pcl_common
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp 
  std_msgs 
  sensor_msgs
  pcl_conversions
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
