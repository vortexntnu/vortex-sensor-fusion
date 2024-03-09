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
                  const cv::Mat& distortion_coeffs,
                  const cv::Ptr<cv::aruco::DetectorParameters> detector_params);
    

    ~ArucoDetector();

    std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, std::vector<int>, std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> detectArucoMarkers(const cv::Mat& input_image);

    std::tuple<int, cv::Vec3d, cv::Vec3d> estimateBoardPose(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> marker_ids);

    cv::Ptr<cv::aruco::Board> createRectangularBoard(float markerSize, float xDist, float yDist, const cv::Ptr<cv::aruco::Dictionary> &dictionary, const std::vector<int> &ids);

    std::vector<int> refineBoardMarkers(const cv::Mat& input_image, std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &rejected_candidates);
    // size_t detectBoardPose();

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    float marker_size_, xDist_, yDist_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Ptr<cv::aruco::Board> board_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

} // namespace vortex::aruco_detector

#endif // ARUCO_DETECTOR_HPP