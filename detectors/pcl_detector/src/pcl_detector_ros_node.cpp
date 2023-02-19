#include "pcl_detector/detectors/euclidean_clustering.hpp"
#include "pcl_detector/pcl_detector_ros.hpp"

using namespace pcl_detector;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_detector");
    ros::NodeHandle nh;

    double tolerance = 1.0; // Meters
    int min_size = 10;
    int max_size = 10000;
    double range = 100; // Meters
    float leaf_size = 0.05; // Meters

    std::shared_ptr<IPclDetector> detector = std::make_shared<EuclideanClusteringDetector>(tolerance, min_size,
        max_size, range);

    PclDetectorRos ros_detector{ nh, detector, leaf_size };

    ros::spin();

    return 0;
}