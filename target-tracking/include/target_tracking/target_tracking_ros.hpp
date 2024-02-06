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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <cmath>

#include <target_tracking/track_manager.hpp>

class TargetTrackingNode : public rclcpp::Node
{
public:
    explicit TargetTrackingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~TargetTrackingNode() {};
    
private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr centroids);

    void timer_callback();

    void update_dyn_model(double std_velocity);

    void update_sensor_model(double std_sensor);

    void update_timer(int update_interval);

    // void parameterCallback(const std::vector<rclcpp::Parameter> &parameters);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    // List of 2d centroids
    mutable std::vector<Eigen::Vector2d> measurements_;

    // Track manager
    TrackManager track_manager_;

    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in_;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Parameter subscriber
    // rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_subscriber_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscriber_;

    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
