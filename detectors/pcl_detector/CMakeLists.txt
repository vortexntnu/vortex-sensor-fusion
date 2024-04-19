cmake_minimum_required(VERSION 3.5) ## 3.8 for cuda?
project(pcl_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(CUDAToolkit)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(${PROJECT_NAME}_node 
  src/pcl_detector_ros_node.cpp 
  src/pcl_detector_ros.cpp 
  src/geometry_processor.cpp
  src/euclidean_clustering.cpp
  src/dbscan_detector.cpp
  src/land_masking.cpp
)

target_link_libraries( ${PROJECT_NAME}_node
  pcl_common
  ${PCL_LIBRARIES}
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp 
  std_msgs 
  sensor_msgs
  pcl_conversions
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# # Conditionally link CUDA libraries
# if(CUDAToolkit_FOUND)
#   set(HAS_CUDA TRUE)
#   target_link_libraries(${PROJECT_NAME}_node CUDA::cudart)
# else()
#   set(HAS_CUDA FALSE)
# endif()

if(CUDAToolkit_FOUND)
  target_sources(${PROJECT_NAME}_node PRIVATE src/pcl_gpu_processor.cpp)
  target_link_libraries(${PROJECT_NAME}_node CUDA::cudart)
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_CUDA)
else()
  target_sources(${PROJECT_NAME}_node PRIVATE src/pcl_cpu_processor.cpp)
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    add_subdirectory(test)
  endif()


ament_package()
