cmake_minimum_required(VERSION 3.0.2)
project(sift)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  rospack
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sift_cpp
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sift_node src/sift_node.cpp)
target_link_libraries(sift ${catkin_LIBRARIES})
# target_link_libraries(sift ${OpenCV_LIBS})

add_executable(orb_node src/orb_node.cpp)
target_link_libraries(orb_node ${catkin_LIBRARIES})


# Local Opencv build from source steps:
# wget https://github.com/opencv/opencv/archive/4.5.3.zip -O opencv-4.5.3.zip
# wget https://github.com/opencv/opencv_contrib/archive/4.5.3.zip -O opencv_contrib-4.5.3.zip
# unzip opencv-4.5.3.zip
# unzip opencv_contrib-4.5.3.zip

# cd opencv-4.5.3
# mkdir build
# cd build
# cmake -D CMAKE_BUILD_TYPE=RELEASE \
#     -D CMAKE_INSTALL_PREFIX=$(realpath ../local_installation) \
#     -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.5.3/modules \
#     -D BUILD_DOCS=OFF \
#     -D BUILD_TESTS=OFF \
#     -D BUILD_PERF_TESTS=OFF \
#     -D BUILD_EXAMPLES=OFF ..
# make -j4
# make install

set(OpenCV_DIR "$ENV{HOME}opencv_4.5.3/opencv-4.5.3/build/local_installation")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(sift ${OpenCV_LIBS})
