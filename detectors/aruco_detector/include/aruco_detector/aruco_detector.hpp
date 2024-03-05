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

    cv::Ptr<cv::aruco::Board> createRectangularBoard(float markerSize, float xDist, float yDist, const cv::Ptr<cv::aruco::Dictionary> &dictionary, const std::vector<int> &ids);

    size_t detectBoardPose(const cv::Mat &originalImg, cv::Mat &modifiedImg, const cv::Ptr<cv::aruco::Board> &board, geometry_msgs::Pose &pose);

private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    float marker_size_, xDist_, yDist_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Ptr<cv::aruco::Board> board_;
};

} // namespace vortex::aruco_detector

#endif // ARUCO_DETECTOR_HPP