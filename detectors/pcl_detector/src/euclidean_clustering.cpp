#include "pcl_detector/detectors/euclidean_clustering.hpp"

EuclideanClusteringDetector::EuclideanClusteringDetector(
    double cluster_tolerance, int min_cluster_size, double range)
    : m_cluster_tolerance{ cluster_tolerance }
    , m_min_cluster_size{ min_cluster_size }
    , m_range{ range }
{
}

pcl::PointCloud<pcl::PointXYZ> EuclideanClusteringDetector::get_detections(
    const pcl::PointCloud<pcl::PointXYZ>& points)
{
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

    for (const auto& indices : cluster_indices) {
        pcl::PointXYZ centroid;
        centroid.getArray3fMap() = Eigen::Array3f::Zero();
        for (const auto& index : indices.indices) {
            const auto& point = points[index];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        centroid.x /= indices.indices.size();
        centroid.y /= indices.indices.size();
        centroid.z /= indices.indices.size();

        if (abs(x) < m_range && abs(y) < m_range && abs(z) < m_range) {
            detections.push_back(pcl::PointXYZ{ x, y, z });
        }
    }

    return detections;
}
