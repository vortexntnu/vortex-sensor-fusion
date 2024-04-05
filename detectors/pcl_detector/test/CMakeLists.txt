find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_test 
    pcl_cpu_processor_test.cpp
    ../src/pcl_cpu_processor.cpp
    ../src/geometry_processor.cpp 
)


target_link_libraries(${PROJECT_NAME}_test
  ${PCL_LIBRARIES} # Link PCL libraries
#   ${OpenCV_LIBS} # Link OpenCV libraries


  # Eigen3::Eigen # Link Eigen3 if your tests require it
  # Add any other libraries your tests might require
)

# target_include_directories(${PROJECT_NAME}_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
#   $<INSTALL_INTERFACE:include>
# #   ${OpenCV_INCLUDE_DIRS} # Ensure OpenCV include dirs are available to the test
# )

target_include_directories(${PROJECT_NAME}_test PUBLIC
  ${PCL_INCLUDE_DIRS} # Add PCL include directories
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_test
#   cv_bridge
)
