#include "pcl_detector/detectors/euclidean_clustering.hpp"
#include "pcl_detector/detectors/dbscan_detector.hpp"
#include "pcl_detector/detectors/optics_detector.hpp"
#include "pcl_detector/detectors/gmm_detector.hpp"
#include "pcl_detector/pcl_detector_ros.hpp"

using namespace pcl_detector;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_detector");
    ros::NodeHandle nh;

    std::shared_ptr<IPclDetector> detector;
    float leaf_size = 0.05; // Meters. Used for downsampling the raw pcl

    // Select the detector based on a command-line argument
    std::string detector_type = "dbscan"; // Default to DBSCAN
    if (argc >= 2) {
        detector_type = argv[1];
    }

    if (detector_type == "dbscan") {
        double eps = 0.5; // Meters
        int min_points = 100;
        detector = std::make_shared<DBSCANDetector>(eps, min_points);
    } else if (detector_type == "optics") {
        double eps = 0.1; // Meters
        int min_points = 2;
        detector = std::make_shared<OPTICSDetector>(eps, min_points);
    } else if (detector_type == "gmm") {
        int num_gaussians = 3;
        int max_iterations = 100;
        double convergence_threshold = 1e-4;
        detector = std::make_shared<GMMDetector>(num_gaussians, max_iterations, convergence_threshold);
    }
    else if (detector_type == "euclidean") {
        double eps = 0.1; // Meters
        int min_points = 100;
        detector = std::make_shared<EuclideanClusteringDetector>(eps, min_points);
    } else {
        ROS_ERROR("Invalid detector type: %s", detector_type.c_str());
        return 1;
    }

    PclDetectorRos ros_detector{ nh, detector, leaf_size };
    ros::spin();

    return 0;
}