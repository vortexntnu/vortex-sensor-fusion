#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>


#include <vortex_msgs/msg/detection_array.hpp>
#include <vortex_msgs/msg/detection.hpp>


class TargetClassifierNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for TargetClassifierNode.
     * 
     * @param options The node options.
     */
    explicit TargetClassifierNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destructor for TargetClassifierNode.
     */
    ~TargetClassifierNode() {};
    
private:
    /**
     * @brief Callback function for the topic subscription.
     * 
     * @param landmark_array The landmark array message.
     */
    void landmark_callback(const vortex_msgs::msg::LandmarkArray::SharedPtr landmark_array);

    void image_detection_callback(const vortex_msgs::msg::DetectionArray::SharedPtr image_detections);

    /**
     * @brief Publishes the landmarks to the output topic.
     * 
     * @param deletion_threshold The deletion threshold for the landmarks.
     */
    void publish_landmarks(double deletion_threshold);

    void update_camera_matrix();

    // Landmark subscriber
    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_subscriber_;
    std::string param_topic_landmarks_in_;

    // Detection subscriber
    rclcpp::Subscription<vortex_msgs::msg::DetectionArray>::SharedPtr detection_subscriber_;
    std::string param_topic_detections_in_;
    
    // Landmark publisher
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_publisher_;
    std::string param_topic_landmarks_out_;

    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Camera matrix
    Eigen::Matrix3d camera_matrix_;

    // Detections
    vortex_msgs::msg::DetectionArray::SharedPtr image_detections_;
};
