#include <aruco_detector/aruco_detector_ros.hpp>

using std::placeholders::_1;

namespace vortex {
namespace aruco_detector {

ArucoDetectorNode::ArucoDetectorNode() : Node("aruco_detector_node")
{
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<std::string>("image_topic", "/image_raw");
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

    this->declare_parameter<float>("aruco.marker_size", 0.150);
    this->declare_parameter<std::string>("aruco.dictionary", "DICT_ARUCO_ORIGINAL");

    this->declare_parameter<bool>("detect_board", true);
    this->declare_parameter<bool>("detect_markers", true);

    this->declare_parameter<float>("board.xDist", 0.430);
    this->declare_parameter<float>("board.yDist", 0.830);
    std::vector<int64_t> board_ids = {28, 7, 96, 19};
    this->declare_parameter("board.ids", board_ids);


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
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(image_topic
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

    this->get_parameter("aruco.marker_size", marker_size_);
    this->get_parameter("board.xDist", xDist_);
    this->get_parameter("board.yDist", yDist_);
    std::vector<int64_t> param_ids;
    this->get_parameter("board.ids", param_ids);
    // Convert int64_t vector to int vector
    std::vector<int> ids_(param_ids.begin(), param_ids.end());



    aruco_detector_ = std::make_unique<ArucoDetector>(dictionary_, marker_size_, camera_matrix_, distortion_coefficients_);

    board_ = aruco_detector_->createRectangularBoard(marker_size_, xDist_, yDist_, dictionary_, ids_);
}



void aruco_detector::ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat input_image = cv_ptr->image;
    geometry_msgs::msg::PoseArray pose_array;

    // DEBUG: Draw detections on image
    auto [marker_corners, marker_ids, rvecs, tvecs] = aruco_detector_->estimatePose(input_image);

    
    for (size_t i = 0; i < marker_ids.size(); i++)
    {
        // Retrieve marker ID and pose
        int marker_id = marker_ids[i];
        cv::Vec3d rvec = rvecs[i];
        cv::Vec3d tvec = tvecs[i];

        tf2::Quaternion quat = rvec_to_quat(rvec);

        auto pose_msg = cv_pose_to_ros_pose_stamped(rvec, quat, frame_, marker_id);
        pose_array.poses.push_back(pose_msg.pose);
    }
        pose_pub_->publish(pose_array);


    cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
    for (size_t i = 0; i < marker_ids.size(); ++i)
    {
        cv::aruco::drawAxis(input_image, camera_matrix_, distortion_coefficients_, rvecs[i], tvecs[i], 0.1);
    }

    auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", input_image).toImageMsg();
    marker_image_pub_->publish(*message);
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

geometry_msgs::msg::PoseStamped aruco_detector::ArucoDetectorNode::cv_pose_to_ros_pose_stamped(const cv::Vec3d &tvec, const tf2::Quaternion &quat, std::string frame_id, int marker_id) {
    // create the PoseStamped message
    geometry_msgs::msg::PoseStamped pose_msg;

    // fill in the header data
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = frame_id + "_marker_id_" +  std::to_string(marker_id);

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
