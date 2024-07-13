#include <wall_tracking/wall_tracking_ros.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallTrackingNode>());
    rclcpp::shutdown();
    return 0;
}