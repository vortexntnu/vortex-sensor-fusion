#include <aruco_detector/aruco_detector.hpp>

namespace vortex::aruco_detector{

ArucoDetector::ArucoDetector(cv::Ptr<cv::aruco::Dictionary> dict,
                             float marker_size, 
                             const cv::Mat& camera_matrix,
                             const cv::Mat& distortion_coeffs)
    : dictionary_(dict), marker_size_{marker_size}, camera_matrix_(camera_matrix), distortion_coeffs_(distortion_coeffs) {
}

ArucoDetector::~ArucoDetector() {}

std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<int>, std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> ArucoDetector::estimatePose(const cv::Mat& input_image) {
    cv::Mat input_image_rgb;
    cv::cvtColor(input_image, input_image_rgb, cv::COLOR_RGBA2RGB);

    cv::Mat input_image_gray;
    cv::cvtColor(input_image_rgb, input_image_gray, cv::COLOR_RGB2GRAY);


    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(input_image_gray, dictionary_, marker_corners, marker_ids);

    std::vector<cv::Vec3d> rvecs, tvecs;

    if (!marker_ids.empty()) {
        cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, distortion_coeffs_, rvecs, tvecs);

    }

    return std::make_tuple(marker_corners, marker_ids, rvecs, tvecs);
}

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
	cv::Ptr<cv::aruco::Board> board = new cv::aruco::Board;
	board                           = cv::aruco::Board::create(markerPoints, dictionary, ids);
	return board;

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

// size_t ArucoDetector::detectBoardPose(const cv::Mat &originalImg, cv::Mat &modifiedImg, const cv::Ptr<cv::aruco::Board> &board, geometry_msgs::Pose &pose)
// {

// 	if (originalImg.empty())
// 		// ROS_WARN("No image");

// 	std::vector<std::vector<cv::Point2f>> corners, rejected;
// 	std::vector<int> ids, recoveredIds;
// 	originalImg.copyTo(modifiedImg);

// 	cv::aruco::detectMarkers(originalImg, board->dictionary, corners, ids, detectorParams, rejected);
// 	if (ids.size() == 0)
// 		return 0;
// 	Draw Markers
// 	cv::aruco::drawDetectedMarkers(modifiedImg, corners, ids);
// 	cv::aruco::drawDetectedMarkers(modifiedImg, rejected, cv::noArray(), cv::Scalar(0, 0, 255));

// 	// Estimate pose
// 	cv::Vec3d rvec, tvec;
// 	int numUsedMarkers = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix, distortionCoefficients, rvec,
// 	                                                  tvec); // replace with cv::solvePnP if OpenCV is updated to v. 4.5.5 or
// 	                                                         // above. It is more accurate
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