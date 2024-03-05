#include <aruco_detector/aruco_detector_ros.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vortex::aruco_detector::ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}