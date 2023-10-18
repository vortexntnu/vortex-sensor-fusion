cmake_minimum_required(VERSION 3.5)
project(subscribers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
# find_package(PCL 1.10 REQUIRED)
# find_package(PCL COMPONENT REQUIRED)
# find_package(PCL REQUIRED)

# include_directories(include ${PCL_INCLUDE_DIRS} )

# target_link_libraries(lidar_sub
#   ${PCL_LIBRARIES}
# )


add_executable(lidar_sub src/lidar_listener.cpp)
ament_target_dependencies(lidar_sub 
  rclcpp 
  std_msgs 
  sensor_msgs 
  pcl_conversions
)
target_link_libraries(lidar_sub 
  pcl_common)
install(TARGETS
  lidar_sub
  DESTINATION lib/${PROJECT_NAME})

# ament_export_include_directories(include)
ament_package()