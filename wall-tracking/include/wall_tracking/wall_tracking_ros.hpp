#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_filtering/filters/pdaf.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <vortex_msgs/msg/parameter.hpp>
#include <vortex_msgs/msg/parameter_array.hpp>
#include <cmath>
#include <wall_tracking/wall_manager.hpp>

class WallTrackingNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for TargetTrackingNode.
     * 
     * @param options The node options.
     */
    explicit WallTrackingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destructor for TargetTrackingNode.
     */
    ~WallTrackingNode() {};
    
private:
    /**
     * @brief Callback function for the topic subscription.
     * 
     * @param clusters The received point cloud data.
     */
    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr walls);

    /**
     * @brief Timer callback function.
     */
    void timer_callback();

    /**
     * @brief Publishes the landmarks to the output topic.
     * 
     * @param deletion_threshold The deletion threshold for the landmarks.
     */
    void publish_landmarks();

    /**
     * @brief Updates the dynamic model with the given velocity standard deviation.
     * 
     * @param std_position The velocity standard deviation.
     */
    void update_dyn_model(double std_position);

    /**
     * @brief Updates the sensor model with the given sensor standard deviation.
     * 
     * @param std_sensor The sensor standard deviation.
     */
    void update_sensor_model(double std_sensor);

    /**
     * @brief Updates the update interval for the target tracking.
     * 
     * @param update_interval The new update interval in milliseconds.
     */
    void update_timer(int update_interval);

    /**
     * @brief Callback function for handling parameter updates.
     * 
     * @param parameters The updated parameters.
     * @return The result of setting the parameters.
     */

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    // List of 2D centroids
    mutable Eigen::Array<double, 4, Eigen::Dynamic> measurements_;

    // Wall manager
    WallManager wall_manager_;

    // Pointcloud subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscriber_;
    std::string param_topic_measurements_in_;
    
    // Landmark publisher
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_publisher_;
    std::string param_topic_landmarks_out_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Parameter subscriber
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscriber_;

    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
