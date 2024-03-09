#ifndef ARUCO_DETECTOR_ROS_HPP
#define ARUCO_DETECTOR_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <map>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cv_bridge/cv_bridge.h>

#include "aruco_detector.hpp"

namespace vortex::aruco_detector
{

static std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionary_map = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};

class ArucoDetectorNode : public rclcpp::Node{

public:
    ArucoDetectorNode();

    ~ArucoDetectorNode(){};

private: 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marker_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr board_image_pub_;

    std::unique_ptr<ArucoDetector> aruco_detector_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    tf2::Quaternion rvec_to_quat(const cv::Vec3d &rvec);

    geometry_msgs::msg::PoseStamped cv_pose_to_ros_pose_stamped(const cv::Vec3d &tvec, const tf2::Quaternion &quat, std::string frame_id, int marker_id);



    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;
    cv::Mat camera_matrix_, distortion_coefficients_;
    bool detect_markers_;
    bool detect_board_;
    float marker_size_,xDist_,yDist_;
    std::vector<int64_t> ids_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::string frame_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;


    
};

} // namespace aruco_detector

#endif //ARUCO_DETECTOR_ROS_HPP