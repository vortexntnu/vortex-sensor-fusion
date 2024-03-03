#include <aruco_detector/aruco_detector_ros.hpp>
#include "aruco_detector_ros.hpp"

using std::placeholders::_1;

namespace vortex::aruco_detector
{

}
ArucoDetectorNode::ArucoDetectorNode()
{

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&ArucoDetectorNode::imageCallback, this, _1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/image_raw/camera_info", 10, std::bind(&ArucoDetectorNode::cameraInfoCallback, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_poses", 10);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_image", 10);
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
    auto [marker_corners, marker_ids, rvecs, tvecs] = aruco_detector->estimatePose(input_image);

    for (int i = 0; i < marker_ids.size(); i++)
    {
        // Retrieve marker ID and pose
        int marker_id = marker_ids[i];
        cv::Vec3d rvec = rvecs[i];
        cv::Vec3d tvec = tvecs[i];

        tf2::Quaternion quat = rvec_to_quat(rvec);

        auto pose_msg = cv_pose_to_ros_pose_stamped(rvec, quat, frame, marker_id);
        pose_array.poses.push_back(pose_msg.pose);
    }
        pose_pub_->publish(pose_array);


    cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
    for (int i = 0; i < marker_ids.size(); ++i)
    {
        cv::aruco::drawAxis(input_image, camera_matrix, distortion_coefficients, rvecs[i], tvecs[i], 0.1);
    }

    sensor_msgs::msg::Image msg_to_publish = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_image).toImageMsg();
    image_publisher.publish(msg_to_publish);
}

tf2::Quaternion rvec_to_quat(const cv::Vec3d &rvec) {
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

geometry_msgs::PoseStamped cv_pose_to_ros_pose_stamped(const cv::Vec3d &tvec, const tf2::Quaternion &quat, std::string frame_id, int marker_id) {
    // create the PoseStamped message
    geometry_msgs::PoseStamped pose_msg;

    // fill in the header data
    pose_msg.header.stamp = ros::Time::now();
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
