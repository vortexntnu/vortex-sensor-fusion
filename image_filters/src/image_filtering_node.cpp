#include <image_filters/image_filtering_ros.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vortex::image_processing::ImageFilteringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
