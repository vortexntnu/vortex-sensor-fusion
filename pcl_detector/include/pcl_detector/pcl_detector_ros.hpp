#pragma once

#include <rclcpp/rclcpp.hpp>  

#include <memory>
#include <iostream>
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include <pcl_detector/detectors/dbscan_detector.hpp>
#include <pcl_detector/detectors/euclidean_clustering.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

enum class DetectorType {
    Euclidean,
    DBSCAN,
};


namespace pcl_detector
{

/**
 * @class pcl_detector::pcl_detector
 * @brief Receives Pointcloud2 message from lidar sensor and filter its points with an optional pcl filter.
 * 
 */
class PclDetectorNode : public rclcpp::Node
{
  public:
    
    /**
     * @brief A constructor for pcl_detector::pcl_detector class
     * @param options Additional options to control creation of the node.
     */
    explicit PclDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief A destructor for PclDetectorNode::PclDetectorNode class
     */
    ~PclDetectorNode() {};

  protected:

    // Initialize the detector
    std::unique_ptr<pcl_detector::IPclDetector> initialize_detector(std::string detector);
  
    /**
     * @brief Use a no filter of pcl library
     * @param msg Pointcloud2 message receveived from the ros2 node
     * @return -
     * @details Omit pointcloud filtering in this example
     */
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

 
    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in_;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;

    std::unique_ptr<pcl_detector::IPclDetector> m_detector;

    bool parameters_changed = false;

    std::unordered_map<std::string, DetectorType> detector_type = {
        { "dbscan", DetectorType::DBSCAN },
        { "euclidean", DetectorType::Euclidean },
    };  
};

} // namespace pcl_detector