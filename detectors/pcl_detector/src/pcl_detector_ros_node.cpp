#include <pcl_detector/pcl_detector_ros.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcl_detector::PclDetectorNode>());
  rclcpp::shutdown();
  return 0;
}