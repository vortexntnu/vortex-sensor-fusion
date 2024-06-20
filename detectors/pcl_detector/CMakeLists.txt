cmake_minimum_required(VERSION 3.26) # 3.8+ required for CUDA
project(pcl_detector LANGUAGES CXX C)

# Set the compiler to GCC 11
set(CMAKE_C_COMPILER /usr/bin/gcc-11)
set(CMAKE_CXX_COMPILER /usr/bin/g++-11)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 for compatibility with CUDA
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Check if CUDA language is available and enable it
include(CheckLanguage)
check_language(CUDA)

if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  # Ensure CUDA is enabled and set correctly
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++17")
endif()

add_executable(${PROJECT_NAME}_node 
  src/pcl_detector_ros_node.cpp 
  src/pcl_detector_ros.cpp 
  src/geometry_processor.cpp
  src/euclidean_clustering.cpp
  src/dbscan_detector.cpp
)

# include_directories(include)
# include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})


# Target include directories
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS} #might need to include custom installation
)

# Target compile features
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)

if(CMAKE_CUDA_COMPILER)
  set_source_files_properties(src/pcl_gpu_processor.cu PROPERTIES LANGUAGE CUDA)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_gpu_processor.cu")
    target_sources(${PROJECT_NAME}_node PRIVATE src/pcl_gpu_processor.cu)
    target_link_libraries(${PROJECT_NAME}_node CUDA::cudart)
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_CUDA)
  else()
    message(FATAL_ERROR "CUDA is found but src/pcl_gpu_processor.cu does not exist.")
  endif()
else()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_cpu_processor.cpp")
    target_sources(${PROJECT_NAME}_node PRIVATE src/pcl_cpu_processor.cpp)
  else()
    message(FATAL_ERROR "CUDA is not found and src/pcl_cpu_processor.cpp does not exist.")
  endif()
endif()

target_link_libraries(${PROJECT_NAME}_node
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
  nav_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Uncomment if you have tests to add
# if(BUILD_TESTING)
#     add_subdirectory(test)
# endif()

ament_package()
