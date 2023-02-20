#include "pcl_detector/detectors/optics_detector.hpp"



namespace pcl_detector {

pcl::PointCloud<pcl::PointXYZ> OPTICSDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points)  {
    pcl::PointCloud<pcl::PointXYZ> detections;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(points.makeShared());

    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
    pcl::OptimalParam param;
    param.eps_ = eps_;
    param.min_pts_ = min_points_;
    pcl::Optics<pcl::PointXYZ> optics(tree, param);
    optics.setInputCloud(points.makeShared());
    optics.getClusters(*clusters);

    for (const auto& cluster : *clusters) {
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
