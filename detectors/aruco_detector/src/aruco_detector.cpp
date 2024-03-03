#include <aruco_detector/aruco_detector.hpp>
#include "aruco_detector.hpp"

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

} // namespace vortex::aruco_detector   