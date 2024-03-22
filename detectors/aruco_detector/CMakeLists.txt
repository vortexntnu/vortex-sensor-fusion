cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

# === C++ standard ===
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  endif()
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED) #OpenCV 4.5.4
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vortex_filtering REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME}_node 
  src/aruco_detector.cpp
  src/aruco_detector_ros.cpp
  src/aruco_detector_node.cpp
  src/aruco_file_logger.cpp
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PUBLIC
${OpenCV_LIBS}
Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  cv_bridge
  vortex_filtering

)

install(TARGETS ${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
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
