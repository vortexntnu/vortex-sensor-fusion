#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <foxglove_msgs/msg/image_annotations.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <target_classifier/auction_algorithm.hpp>

#include <vortex_msgs/msg/detection.hpp>
#include <vortex_msgs/msg/detection_array.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
// #include "std_msgs/msg/header.h"
// #include "std_msgs/"
// #include "geometry_msgs/Point.h"
// #include "builtin_interfaces/msg/duration.hpp"


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

    void subscribeToCameraTopics();

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

    void update_camera_matrix();

    void timer_callback();

    void visualize_landmark_pixels(const std::vector<Eigen::Vector3d>& landmark_pixels);

    geometry_msgs::msg::TransformStamped z_up_to_z_fwd(geometry_msgs::msg::TransformStamped transform_stamped);

    std::vector<Eigen::Vector3d> get_pixel_coordinates(const geometry_msgs::msg::TransformStamped transform);

    Eigen::MatrixXd generate_reward_matrix(const vortex_msgs::msg::DetectionArray::SharedPtr image_detections, const std::vector<Eigen::Vector3d>& landmark_pixels);

    // Landmark subscriber
    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_subscriber_;
    std::string param_topic_landmarks_in_;

    // Detection subscriber
    rclcpp::Subscription<vortex_msgs::msg::DetectionArray>::SharedPtr detection_subscriber_;
    std::string param_topic_detections_in_;

    // Camera parameter subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    std::string param_topic_camera_info_;
    
    // Landmark publisher
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_publisher_;
    std::string param_topic_landmarks_out_;

    // Landmark pixel coordinate publisher
    rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr landmark_pixel_publisher_;
    std::string param_topic_landmark_pixel_out_;

    //detection publisher
    rclcpp::Publisher<visualization_msgs::msg::ImageMarker>::SharedPtr detection_publisher_;
    std::string param_topic_detections_out_;

    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Camera matrix
    Eigen::Matrix3d camera_matrix_;

    // Detections
    vortex_msgs::msg::DetectionArray::SharedPtr image_detections_;

    // Landmarks
    vortex_msgs::msg::LandmarkArray::SharedPtr landmarks_;

    // Camera info topic
    std::string camera_info_topic_;
};
