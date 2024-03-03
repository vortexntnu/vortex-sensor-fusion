#include <aruco_detector/aruco_detector.hpp>

namespace vortex::aruco_detector{

ArucoDetector::ArucoDetector(cv::Ptr<cv::aruco::Dictionary> dict,
                             float marker_size, 
                             const cv::Mat& camera_matrix,
                             const cv::Mat& distortion_coeffs)
    : dictionary(dict), marker_size{marker_size}, camera_matrix(camera_matrix), distortion_coeffs(distortion_coeffs) {
}

ArucoDetector::~ArucoDetector() {}

std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<int>, std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> ArucoDetector::estimatePose(const cv::Mat& input_image) {
    cv::Mat input_image_rgb;
    cv::cvtColor(input_image, input_image_rgb, cv::COLOR_RGBA2RGB);

    cv::Mat input_image_gray;
    cv::cvtColor(input_image_rgb, input_image_gray, cv::COLOR_RGB2GRAY);


    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(input_image_gray, dictionary, marker_corners, marker_ids);

    std::vector<cv::Vec3d> rvecs, tvecs;

    if (!marker_ids.empty()) {
        cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size, camera_matrix, distortion_coeffs, rvecs, tvecs);

    }

    return std::make_tuple(marker_corners, marker_ids, rvecs, tvecs);
}
} // namespace vortex::aruco_detector   