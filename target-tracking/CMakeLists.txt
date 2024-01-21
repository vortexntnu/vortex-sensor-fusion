cmake_minimum_required(VERSION 3.5)
project(target_tracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vortex_filtering REQUIRED)
find_package(vortex_msgs REQUIRED)

include_directories(include)


add_executable(${PROJECT_NAME}_node 
  src/target_tracking_ros_node.cpp 
  src/target_tracking_ros.cpp 
  src/track_manager.cpp
)

target_link_libraries( ${PROJECT_NAME}_node
  pcl_common
  Eigen3::Eigen
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp 
  std_msgs 
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
  vortex_filtering
  vortex_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
