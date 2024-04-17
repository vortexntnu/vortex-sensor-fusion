cmake_minimum_required(VERSION 3.8)
project(image_filtering)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)


include_directories(include)


add_executable(${PROJECT_NAME}_node 
  src/image_processing.cpp
  src/image_filtering_ros.cpp
  src/image_filtering_node.cpp
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PUBLIC
${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  rclcpp
  cv_bridge
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
