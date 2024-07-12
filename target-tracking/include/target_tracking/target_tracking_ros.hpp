#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_filtering/filters/pdaf.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/clusters.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <vortex_msgs/msg/parameter.hpp>
#include <vortex_msgs/msg/parameter_array.hpp>
#include <cmath>
#include <target_tracking/track_manager.hpp>

class TargetTrackingNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for TargetTrackingNode.
     * 
     * @param options The node options.
     */
    explicit TargetTrackingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destructor for TargetTrackingNode.
     */
    ~TargetTrackingNode() {};
    
private:
    /**
     * @brief Callback function for the topic subscription.
     * 
     * @param clusters The received point cloud data.
     */
    void topic_callback(const vortex_msgs::msg::Clusters::SharedPtr clusters);

    /**
     * @brief Timer callback function.
     */
    void timer_callback();

    /**
     * @brief Publishes the landmarks to the output topic.
     * 
     * @param deletion_threshold The deletion threshold for the landmarks.
     */
    void publish_landmarks(double deletion_threshold);

    /**
     * @brief Publishes the visualization parameters to the output topic.
     * 
     * @param gate_threshold The gating threshold.
     * 
     * @param gate_min_threshold The minimum gating threshold.
     * 
     * @param gate_max_threshold The maximum gating threshold.
     */
    void publish_visualization_parameters(double gate_threshold, double gate_min_threshold, double gate_max_threshold);

    /**
     * @brief Updates the dynamic model with the given velocity standard deviation.
     * 
     * @param std_velocity The velocity standard deviation.
     */
    void update_dyn_model(double std_velocity);

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
    mutable Eigen::Array<double, 2, Eigen::Dynamic> measurements_;

    mutable std::vector<float> centroid_z_meas_;

    mutable std::vector<std::vector<Eigen::Vector3f>> clusters_;

    // Track manager
    TrackManager track_manager_;

    // Pointcloud subscriber
    rclcpp::Subscription<vortex_msgs::msg::Clusters>::SharedPtr subscriber_;
    std::string param_topic_pointcloud_in_;
    
    // Landmark publisher
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_publisher_;
    std::string param_topic_landmarks_out_;

    // Visualization publisher
    rclcpp::Publisher<vortex_msgs::msg::ParameterArray>::SharedPtr visualization_publisher_;
    std::string param_topic_visualization_out_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Parameter subscriber
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscriber_;

    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
