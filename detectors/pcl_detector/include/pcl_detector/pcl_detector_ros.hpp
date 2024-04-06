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

#include <pcl_detector/pcl_processor.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>



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

    geometry_msgs::msg::PoseArray getWallPoses(std::vector<pcl::PointXYZ> wall_poses);

    void transformLines(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg, std::vector<Eigen::VectorXf>& prev_lines);


    std::tuple<Eigen::Vector3f, Eigen::Quaternionf> calculateTransformation(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);

    void processLine(const Eigen::VectorXf& line, std::vector<int> inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void publishLineMarkerArray(const std::vector<Eigen::VectorXf>& lines, const std::string& frame_id);
    void publishtfLineMarkerArray(const std::vector<Eigen::VectorXf>& lines, const std::string& frame_id);

    void publishWallMarkerArray(const geometry_msgs::msg::PoseArray& pose_array, const std::string& frame_id);
    geometry_msgs::msg::Point ExtendLineFromOriginToLength(const geometry_msgs::msg::Point& point, double length);
    void publishExtendedLinesFromOrigin(const geometry_msgs::msg::PoseArray& pose_array, const std::string& frame_id);






 
    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in_;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tf_line_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_cone_publisher;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_publisher_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;

    std::unique_ptr<pcl_detector::IPclDetector> detector_;

    std::unique_ptr<PclProcessor> processor_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool parameters_changed_ = false;

    std::vector<Eigen::VectorXf> current_lines_;
    std::vector<Eigen::VectorXf> prev_lines_;
    geometry_msgs::msg::PoseArray wall_poses_;

    std::unordered_map<std::string, DetectorType> detector_type = {
        { "dbscan", DetectorType::DBSCAN },
        { "euclidean", DetectorType::Euclidean },
    };  
};

} // namespace pcl_detector