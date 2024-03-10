#include <aruco_detector/aruco_detector.hpp>
#include <iostream>

namespace vortex::aruco_detector{

ArucoDetector::ArucoDetector(cv::Ptr<cv::aruco::Dictionary> dict,
                             float marker_size, 
                             const cv::Mat& camera_matrix,
                             const cv::Mat& distortion_coeffs,
							 const cv::Ptr<cv::aruco::DetectorParameters> detector_params)
    : dictionary_(dict), marker_size_{marker_size}, camera_matrix_(camera_matrix), distortion_coeffs_(distortion_coeffs), detector_params_{detector_params} {
}

ArucoDetector::~ArucoDetector() {}

cv::Ptr<cv::aruco::Board> ArucoDetector::createRectangularBoard(float markerSize, float xDist, float yDist, const cv::Ptr<cv::aruco::Dictionary> &dictionary,
                                                               const std::vector<int> &ids)
{
	const float markerHalf{markerSize / 2}, xHalf{xDist / 2}, yHalf{yDist / 2};

	// Define center of each marker
	std::vector<cv::Point3f> markerCenters;
	markerCenters.push_back({-xHalf - markerHalf, yHalf + markerHalf, 0});
	markerCenters.push_back({xHalf + markerHalf, yHalf + markerHalf, 0});
	markerCenters.push_back({xHalf + markerHalf, -yHalf - markerHalf, 0});
	markerCenters.push_back({-xHalf - markerHalf, -yHalf - markerHalf, 0});

	// Place marker at each marker center
	std::vector<std::vector<cv::Point3f>> markerPoints;
	for (size_t i{0}; i < markerCenters.size(); i++) {
		std::vector<cv::Point3f> marker;
		float xOffset{markerCenters.at(i).x};
		float yOffset{markerCenters.at(i).y};

		// Marker corners need to be added CLOCKWISE from top left corner
		marker.push_back({xOffset - markerHalf, yOffset + markerHalf, 0});
		marker.push_back({xOffset + markerHalf, yOffset + markerHalf, 0});
		marker.push_back({xOffset + markerHalf, yOffset - markerHalf, 0});
		marker.push_back({xOffset - markerHalf, yOffset - markerHalf, 0});
		markerPoints.push_back(marker);
	}
	cv::Ptr<cv::aruco::Board> board_ = new cv::aruco::Board;
	board_                           = cv::aruco::Board::create(markerPoints, dictionary, ids);
	return board_;

	

	/*

	top left corner of marker
	|
	X---O               X---O
	|id0|---- xDist ----|id1|
	O---O               O---O
	  |                   |
	  |                   |
	  |                   |
	  |                   |
	yDist       O       yDist
	  |         |         |
	  |      origin       |
	  |                   |
	  |                   |
	X---O               X---O
	|id3|---- xDist ----|id2|
	O---O               O---O
	|   |
	markerSize

	*/
};

std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, std::vector<int>> ArucoDetector::detectArucoMarkers(const cv::Mat &input_image)
{
 
	std::vector<int> marker_ids;
	std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

	
	cv::aruco::detectMarkers(input_image, dictionary_, marker_corners, marker_ids, detector_params_, rejected_candidates);

	std::vector<cv::Vec3d> rvecs, tvecs;
	// std::cout << "marker_ids.size(): " << marker_ids.size() << std::endl;
	// std::cout << "rejected_candidates.size(): " << rejected_candidates.size() << std::endl;
	// if (!marker_ids.empty()) {
		cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, distortion_coeffs_, rvecs, tvecs);

	// }

    return std::make_tuple(marker_corners, rejected_candidates, marker_ids);
}

    std::tuple<int, cv::Vec3d, cv::Vec3d> ArucoDetector::estimateBoardPose(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> marker_ids){
		cv::Vec3d rvec, tvec;
		int numUsedMarkers = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, board_, camera_matrix_, distortion_coeffs_, rvec, tvec);

		return {numUsedMarkers, rvec, tvec};
	}


	std::vector<int> ArucoDetector::refineBoardMarkers(const cv::Mat &input_image, std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &marker_ids, std::vector<std::vector<cv::Point2f>> &rejected_candidates)
{
	std::vector<int> recoveredIds;
	cv::aruco::refineDetectedMarkers(input_image, board_, corners, marker_ids, rejected_candidates, camera_matrix_, distortion_coeffs_, 10, 3, true, recoveredIds, detector_params_);
	return recoveredIds;
}



// size_t ArucoDetector::detectBoardPose(const cv::Mat &originalImg, cv::Mat &modifiedImg, const cv::Ptr<cv::aruco::Board> &board, geometry_msgs::Pose &pose)
// {
// 	cv::aruco::refineDetectedMarkers(originalImg, board, corners, ids, rejected, cameraMatrix, distortionCoefficients, 10, 3, true, recoveredIds, detectorParams);
// 	numUsedMarkers += recoveredIds.size();
// 	size_t detectedMarkerThreshhold = 1;
// 	if (numUsedMarkers < 1)
// 		return 0; // Don't estimate if you only see less markers than the threshhold
// 	pose = tvec_rvec2pose(rvec, tvec);

// 	float length = cv::norm(board->objPoints[0][0] - board->objPoints[0][1]); // Visual length of the drawn axis
// 	cv::aruco::drawAxis(modifiedImg, cameraMatrix, distortionCoefficients, rvec, tvec, length);

// 	return numUsedMarkers;
// }

} // namespace vortex::aruco_detector   