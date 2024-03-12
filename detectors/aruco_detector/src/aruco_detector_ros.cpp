// #include <aruco_detector/aruco_detector_ros.hpp>
#include "../include/aruco_detector/aruco_detector_ros.hpp"


using std::placeholders::_1;

namespace vortex {
namespace aruco_detector {

ArucoDetectorNode::ArucoDetectorNode() : Node("aruco_detector_node")
{
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<std::string>("image_topic", "/flir_camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/image_raw/camera_info");

    this->declare_parameter<float>("camera.fx", 1061.29517988700);
    this->declare_parameter<float>("camera.fy", 1061.17169143453);
    this->declare_parameter<float>("camera.cx", 723.504570055496);
    this->declare_parameter<float>("camera.cy", 585.265411909955);
    this->declare_parameter<float>("distortion.k1", 0.000335365051980971);
    this->declare_parameter<float>("distortion.k2", 0.000583836572965934);
    this->declare_parameter<float>("distortion.p1", 0.0);
    this->declare_parameter<float>("distortion.p2", 0.0);
    this->declare_parameter<float>("distortion.k3", 0.000318839213604595);

    this->declare_parameter<float>("aruco.marker_size", 0.167);
    this->declare_parameter<std::string>("aruco.dictionary", "DICT_5X5_250");

    this->declare_parameter<bool>("detect_board", true);
    this->declare_parameter<bool>("detect_markers", true);
    this->declare_parameter<bool>("visualize", true);

    this->declare_parameter<float>("board.xDist", 0.462);
    this->declare_parameter<float>("board.yDist", 0.862);
    std::vector<int64_t> board_ids = {28, 7, 96, 19};
    this->declare_parameter("board.ids", board_ids);

    this->declare_parameter("board.dynmod.stddev", 0.01);
    this->declare_parameter("board.sensmod.stddev", 0.01);

    // Retrieve parameters
    float fx, fy, cx, cy, k1, k2, p1, p2, k3;
    this->get_parameter("camera.fx", fx);
    this->get_parameter("camera.fy", fy);
    this->get_parameter("camera.cx", cx);
    this->get_parameter("camera.cy", cy);
    this->get_parameter("distortion.k1", k1);
    this->get_parameter("distortion.k2", k2);
    this->get_parameter("distortion.p1", p1);
    this->get_parameter("distortion.p2", p2);
    this->get_parameter("distortion.k3", k3);


    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    distortion_coefficients_ = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    
    std::string image_topic,camera_info_topic;
    this->get_parameter("image_topic", image_topic);
    this->get_parameter("camera_info_topic", camera_info_topic);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_topic"
    , 10, std::bind(&ArucoDetectorNode::imageCallback, this, _1));
    // camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10, std::bind(&ArucoDetectorNode::cameraInfoCallback, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_poses", 10);

    marker_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_marker_image", 10);

    board_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_board_image", 10);
    

    this->get_parameter("camera_frame",frame_);

    std::string dictionary_type = this->get_parameter("aruco.dictionary").as_string();
    if(dictionary_map.count(dictionary_type)) {
        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_map[dictionary_type]);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid dictionary type received: %s. Using default.", dictionary_type.c_str());
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    }

    this->get_parameter("detect_markers", detect_markers_);
    this->get_parameter("detect_board", detect_board_);
    this->get_parameter("visualize", visualize_);


    this->get_parameter("aruco.marker_size", marker_size_);
    this->get_parameter("board.xDist", xDist_);
    this->get_parameter("board.yDist", yDist_);
    std::vector<int64_t> param_ids;
    this->get_parameter("board.ids", param_ids);
    // Convert int64_t vector to int vector
    std::vector<int> ids_(param_ids.begin(), param_ids.end());

    double dynmod_stddev = this->get_parameter("board.dynmod.stddev").as_double();
    double sensmod_stddev = this->get_parameter("board.sensmod.stddev").as_double();

    dynamic_model_ = std::make_shared<DynMod>(dynmod_stddev);
    sensor_model_ = std::make_shared<SensMod>(sensmod_stddev);

    detector_params_ = cv::aruco::DetectorParameters::create();
	detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;


    aruco_detector_ = std::make_unique<ArucoDetector>(dictionary_, marker_size_, camera_matrix_, distortion_coefficients_, detector_params_);

    board_ = aruco_detector_->createRectangularBoard(marker_size_, xDist_, yDist_, dictionary_, ids_);

    if(detect_board_){
        auto timer_callback = std::bind(&ArucoDetectorNode::kalmanFilterCallback, this);
        auto timer = this->create_wall_timer(std::chrono::milliseconds(10), timer_callback); 
    }

}

void aruco_detector::ArucoDetectorNode::kalmanFilterCallback()
{
  static rclcpp::Time previous_time = this->now();
    rclcpp::Time current_time = this->now();
    rclcpp::Duration time_since_previous_callback = current_time - previous_time;
    previous_time = current_time;

  auto [status, board_pose_meas, stamp] = board_measurement_.getBoardPoseStamp();
  switch(status) {
    case BoardDetectionStatus::BOARD_NEVER_DETECTED:
      return;
    case BoardDetectionStatus::MEASUREMENT_AVAILABLE:
      std::tie(board_pose_est_, std::ignore, std::ignore) = EKF::step(dynamic_model_, sensor_model_, time_since_previous_callback.seconds(),board_pose_est_, board_pose_meas);
      break;
    case BoardDetectionStatus::MEASUREMENT_NOT_AVAILABLE:
      std::tie(board_pose_est_, std::ignore) = EKF::predict(dynamic_model_, sensor_model_, time_since_previous_callback.seconds(), board_pose_est_);
      break;
  }
    

    
  }


void aruco_detector::ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

    cv_bridge::CvImagePtr cv_ptr;
        cv::Mat input_image_gray;
        cv::Mat input_image_rgb;
        cv::Mat input_image;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        input_image = cv_ptr->image;

        cv::cvtColor(input_image, input_image_rgb, cv::COLOR_RGBA2RGB);

	    cv::cvtColor(input_image_rgb, input_image_gray, cv::COLOR_RGB2GRAY);

    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "cv_bridge exception: " << e.what());

        // RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }



    // DEBUG: Draw detections on image
  
    // DEBUG: Draw detections on image
    // markers to vector transform will be done after refineBoardMarkers in case of recovered candidates
    auto [marker_corners, rejected_candidates, marker_ids] = aruco_detector_->detectArucoMarkers(input_image_gray);

    cv::Vec3d board_rvec, board_tvec;
    if(detect_board_ && marker_ids.size() > 0){
    auto [valid, board_rvec, board_tvec] = aruco_detector_->estimateBoardPose(marker_corners, marker_ids);
    if (valid > 0) {
        Eigen::Vector<double,6> pose(6);
        pose << board_tvec[0], board_tvec[1], board_tvec[2], board_rvec[0], board_rvec[1], board_rvec[2];
        if(std::get<0>(board_measurement_.getBoardPoseStamp()) == BoardDetectionStatus::BOARD_NEVER_DETECTED){
            board_pose_est_ = {pose,Eigen::Matrix<double,6,6>::Identity()};
        }
        else { 
        rclcpp::Time stamp = msg->header.stamp;
        board_measurement_.setBoardPoseStamp(std::make_tuple(BoardDetectionStatus::MEASUREMENT_AVAILABLE, pose, stamp));

        std::vector<int> recovered_candidates = aruco_detector_->refineBoardMarkers(input_image_gray, marker_corners, marker_ids, rejected_candidates);
           
        }
    }
    else {
        board_measurement_.setBoardStatus(BoardDetectionStatus::MEASUREMENT_NOT_AVAILABLE);
    }
    }


    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, distortion_coefficients_, rvecs, tvecs);

    geometry_msgs::msg::PoseArray pose_array;

    for (size_t i = 0; i < marker_ids.size(); i++)
    {
        cv::Vec3d rvec = rvecs[i];
        // cv::Vec3d tvec = tvecs[i];

        tf2::Quaternion quat = rvec_to_quat(rvec);

        auto pose_msg = cv_pose_to_ros_pose_stamped(rvec, quat, frame_, msg->header.stamp);
        pose_array.poses.push_back(pose_msg.pose);
    }

        pose_array.header.stamp = msg->header.stamp;
        pose_array.header.frame_id = frame_;
        pose_pub_->publish(pose_array);

    if(visualize_){

    cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
    cv::aruco::drawDetectedMarkers(input_image, rejected_candidates, cv::noArray(), cv::Scalar(100, 0, 255));
    
    for (size_t i = 0; i < marker_ids.size(); ++i){
        cv::aruco::drawAxis(input_image, camera_matrix_, distortion_coefficients_, rvecs[i], tvecs[i], 0.1);
    }


    auto message = cv_bridge::CvImage(msg->header, "bgr8", input_image).toImageMsg();

    if(detect_board_){
    float length = cv::norm(board_->objPoints[0][0] - board_->objPoints[0][1]); // Visual length of the drawn axis
	cv::aruco::drawAxis(input_image, camera_matrix_, distortion_coefficients_, board_rvec, board_tvec, length);
    board_image_pub_->publish(*message);
    }

    marker_image_pub_->publish(*message);
}
}

tf2::Quaternion aruco_detector::ArucoDetectorNode::rvec_to_quat(const cv::Vec3d &rvec) {
    // Convert rotation vector to rotation matrix
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    
    // Convert rotation matrix to quaternion
    tf2::Matrix3x3 rotation_matrix(
        rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
        rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
        rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2)
    );

    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    return quaternion;
}

geometry_msgs::msg::PoseStamped aruco_detector::ArucoDetectorNode::cv_pose_to_ros_pose_stamped(const cv::Vec3d &tvec, const tf2::Quaternion &quat, std::string frame_id, rclcpp::Time stamp) {
    // create the PoseStamped message
    geometry_msgs::msg::PoseStamped pose_msg;

    // fill in the header data
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id;

    // fill in the position data
    pose_msg.pose.position.x = tvec[0];
    pose_msg.pose.position.y = tvec[1];
    pose_msg.pose.position.z = tvec[2];

    // fill in the orientation data
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    // publish the PoseStamped message
    return pose_msg;
}
} // namespace vortex::aruco_detector
} // namespace vortex
