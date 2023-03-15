#include "pcl_detector/detectors/dbscan_detector.hpp"
#include "pcl_detector/detectors/euclidean_clustering.hpp"
#include "pcl_detector/detectors/gmm_detector.hpp"
#include "pcl_detector/detectors/optics_detector.hpp"
#include "pcl_detector/pcl_detector_ros.hpp"

using namespace pcl_detector;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_detector");
    ros::NodeHandle nh;

    PclDetectorRos ros_detector{ nh };
    ros::spin();

    return 0;
}