#include "pcl_detector/detectors/dbscan_detector.hpp"
#include "pcl_detector/pcl_detector_ros.hpp"

using namespace pcl_detector;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_detector");
    ros::NodeHandle nh;

    double eps = 1.0; // Meters
    int min_points = 10;
    float leaf_size = 0.05; // Meters

    std::shared_ptr<IPclDetector> detector = std::make_shared<DBSCANDetector>(eps, min_points);

    PclDetectorRos ros_detector{ nh, detector, leaf_size };

    ros::spin();

    return 0;
}