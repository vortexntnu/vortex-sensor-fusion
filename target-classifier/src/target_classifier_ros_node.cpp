#include <target_classifier/target_classifier_ros.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetClassifierNode>());
    rclcpp::shutdown();
    return 0;
}