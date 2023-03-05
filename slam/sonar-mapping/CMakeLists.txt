cmake_minimum_required(VERSION 2.8.3)
project(sonar_mapping_c)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   eigen_conversions
   nav_msgs
   roscpp
   sensor_msgs
   std_msgs
   tf2
   tf2_geometry_msgs
   tf2_ros
   vortex_msgs
  
)

## Adding EIGEN
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

catkin_package(
   INCLUDE_DIRS
      include
   CATKIN_DEPENDS
      eigen_conversions
      nav_msgs
      roscpp
      sensor_msgs
      std_msgs
      tf2
      tf2_geometry_msgs
      tf2_ros
      vortex_msgs
      
   DEPENDS
      ${EIGEN_PACKAGE}
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})


add_library(math_functions src/math_functions.cpp)


add_executable(${PROJECT_NAME}_node src/sonar_mapping_node.cpp)


target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} math_functions)
