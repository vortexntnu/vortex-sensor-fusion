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

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <target_tracking/track_manager.hpp>

class TargetTrackingNode : public rclcpp::Node
{
public:
    explicit TargetTrackingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~TargetTrackingNode() {};
    
private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr centroids);

    void timer_callback();

    // List of 2d centroids
    mutable std::vector<Eigen::Vector2d> measurements_;

    // Track manager
    TrackManager track_manager_;

    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in_;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
