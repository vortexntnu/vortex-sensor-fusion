#include "pcl_detector/detectors/dbscan_detector.hpp"



namespace pcl_detector {

pcl::PointCloud<pcl::PointXYZ> DBSCANDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) {
    pcl::PointCloud<pcl::PointXYZ> detections;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(points.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(eps_);
    ec.setMinClusterSize(min_points_);
    ec.setMaxClusterSize(std::numeric_limits<int>::max());
    ec.setSearchMethod(tree);
    ec.setInputCloud(points.makeShared());
    ec.extract(cluster_indices);

    for (auto& indices : cluster_indices) {
        for (auto& index : indices.indices) {
            detections.push_back(points[index]);
        }
    }

    return detections;
}

}; // namespace pcl_detector