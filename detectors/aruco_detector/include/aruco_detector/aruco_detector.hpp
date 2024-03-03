#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace vortex::aruco_detector
{
    
      
class ArucoDetector {
public:
    ArucoDetector(cv::Ptr<cv::aruco::Dictionary> dict,
                  float marker_size, 
                  const cv::Mat& camera_matrix,
                  const cv::Mat& distortion_coeffs);

    ~ArucoDetector();

    std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<int>, std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> estimatePose(const cv::Mat& input_image);

private:
    float marker_size;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
};

} // namespace vortex::aruco_detector

#endif // ARUCO_DETECTOR_HPP