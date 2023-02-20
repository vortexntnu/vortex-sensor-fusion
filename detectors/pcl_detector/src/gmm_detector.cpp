#include "pcl_detector/detectors/gmm_detector.hpp"

namespace pcl_detector {

pcl::PointCloud<pcl::PointXYZ> GMMDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) {
    pcl::PointCloud<pcl::PointXYZ> detections;

    pcl::GaussianMixtureModel<pcl::PointXYZ> gmm(num_gaussians_);
    gmm.setInputCloud(points.makeShared());
    gmm.setConvergenceThreshold(convergence_threshold_);
    gmm.setClusteringMaxIterations(max_iterations_);
    gmm.setProbabilityThreshold(1e-5);
    gmm.compute();

    pcl::IndicesClusters clusters;
    gmm.getClusters(clusters);

    for (const auto& cluster : clusters) {
        pcl::PointXYZ centroid;
        centroid.getArray3fMap() = Eigen::Array3f::Zero();
        for (const auto& index : cluster.indices) {
            const auto& point = points[index];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        centroid.x /= cluster.indices.size();
        centroid.y /= cluster.indices.size();
        centroid.z /= cluster.indices.size();
        detections.push_back(centroid);
    }

    return detections;
}

}; // namespace pcl_detector
