#include "pcl_detector/detectors/euclidean_clustering.hpp"

EuclideanClusteringDetector::EuclideanClusteringDetector(
    double cluster_tolerance, int min_cluster_size, int max_cluster_size,
    double range)
    : m_cluster_tolerance{ cluster_tolerance }
    , m_min_cluster_size{ min_cluster_size }
    , m_max_cluster_size{ max_cluster_size }
    , m_range{ range }
{
}

pcl::PointCloud<pcl::PointXYZ> EuclideanClusteringDetector::get_detections(
    const pcl::PointCloud<pcl::PointXYZ>& points)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids{ new pcl::PointCloud<pcl::PointXYZ> };

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(points.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(m_cluster_tolerance);
    ec.setMinClusterSize(m_min_cluster_size);
    ec.setMaxClusterSize(m_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(points.makeShared());

    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        float x, y, z = 0;
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            x += points.points[*pit].x;
            y += points.points[*pit].y;
            z += points.points[*pit].z;
        }

        x /= it->indices.size();
        y /= it->indices.size();
        z /= it->indices.size();

        if (abs(x) < m_range && abs(y) < m_range && abs(z) < m_range) {
            centroids->push_back(pcl::PointXYZ{ x, y, z });
        }
    }

    return *centroids;
}
