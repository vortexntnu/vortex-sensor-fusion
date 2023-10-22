cmake_minimum_required(VERSION 3.5)
project(subscribers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Add PCL to your package dependencies
find_package(PCL REQUIRED)

add_executable(lidar_sub src/lidar_listener.cpp)
ament_target_dependencies(lidar_sub 
  rclcpp 
  std_msgs 
  sensor_msgs 
  pcl_conversions
)

# Link against PCL libraries
target_link_libraries(lidar_sub 
  ${PCL_LIBRARIES}
)

install(TARGETS
  lidar_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
