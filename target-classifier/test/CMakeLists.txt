find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_test 
  gtest_main.cpp
  auction_algorithm_test.cpp
  ../src/auction_algorithm.cpp
)
target_include_directories(${PROJECT_NAME}_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_test
  std_msgs
)
target_link_libraries(${PROJECT_NAME}_test 
  Eigen3::Eigen # Makes us able to use #include <Eigen/Dense> instead of #include <eigen3/Eigen/Dense>
  OpenMP::OpenMP_CXX
)