#ifndef ARUCO_DETECTOR_ROS_HPP
#define ARUCO_DETECTOR_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/parameter_event_handler.hpp"
#include <rclcpp/qos.hpp>
#include <mutex>
#include <tuple>

#include <map>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <vortex_filtering/vortex_filtering.hpp>

#include <cv_bridge/cv_bridge.h>

#include "aruco_detector.hpp"

namespace vortex::aruco_detector
{
using Vector6d = Eigen::Vector<double, 6>;

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
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marker_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr board_pose_pub_;

    void setCameraParams();

    void initializeDetector();

    void setVisualization();

    void setBoardDetection();

    void initializeBoard();

    void initializeModels();

    void toggleKalmanFilterCallback();

    void checkAndSubscribeToCameraTopics();

    void setFrame();
    
    void initializeParameterHandler();
    
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    rclcpp::ParameterEventCallbackHandle::SharedPtr param_cb_handle_;
    void onParameterEvent(const rcl_interfaces::msg::ParameterEvent &event);

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void kalmanFilterCallback();

    tf2::Quaternion rvec_to_quat(const cv::Vec3d &rvec);

    geometry_msgs::msg::PoseStamped cv_pose_to_ros_pose_stamped(const cv::Vec3d &tvec, const tf2::Quaternion &quat, std::string frame_id, rclcpp::Time stamp);


    using DynMod = vortex::models::IdentityDynamicModel<6>;
    using SensMod = vortex::models::IdentitySensorModel<6,6>;
    using EKF = vortex::filter::EKF<DynMod, SensMod>;

    enum class BoardDetectionStatus{
        BOARD_NEVER_DETECTED,
        MEASUREMENT_AVAILABLE,
        MEASUREMENT_NOT_AVAILABLE,
    };

    struct BoardPoseStamp{
    std::tuple<BoardDetectionStatus, Vector6d, rclcpp::Time> getBoardPoseStamp() const {
        std::lock_guard<std::mutex> lock(struct_mutex_);
        return {board_detection_status_, board_pose_meas_, stamp_};
    }

    void setBoardStatus(BoardDetectionStatus status) {
        std::lock_guard<std::mutex> lock(struct_mutex_);
        board_detection_status_ = status;
    }
    
    void setBoardPoseStamp(const std::tuple<BoardDetectionStatus, Vector6d, rclcpp::Time>& values) {
        std::lock_guard<std::mutex> lock(struct_mutex_);
        board_detection_status_ = std::get<0>(values);
        board_pose_meas_ = std::get<1>(values);
        stamp_ = std::get<2>(values);
    }
    
    private:
        mutable std::mutex struct_mutex_;
        BoardDetectionStatus board_detection_status_ = BoardDetectionStatus::BOARD_NEVER_DETECTED;
        Vector6d board_pose_meas_ = Vector6d::Zero(6); 
        rclcpp::Time stamp_;
    };
    
    std::unique_ptr<ArucoDetector> aruco_detector_;

    cv::Mat camera_matrix_, distortion_coefficients_;

    bool detect_board_;
    bool visualize_;

    float marker_size_;
    float xDist_,yDist_;
    std::vector<int64_t> ids_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::string frame_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Board> board_;
    BoardPoseStamp board_measurement_;
    std::shared_ptr<DynMod> dynamic_model_;
    std::shared_ptr<SensMod> sensor_model_;
    vortex::prob::Gauss<6> board_pose_est_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    std::string image_topic_;
    std::string camera_info_topic_;

};
} // namespace aruco_detector

#endif //ARUCO_DETECTOR_ROS_HPP