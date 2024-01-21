#include <target_tracking/target_tracking_ros.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetTrackingNode>());
    rclcpp::shutdown();
    return 0;
}