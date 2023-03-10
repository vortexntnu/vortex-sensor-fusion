#ifndef EUCLIDEAN_CLUSTERING_H
#define EUCLIDEAN_CLUSTERING_H

#include "pcl_detector/pcl_detector.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

class EuclideanClusteringDetector : public pcl_detector::IPclDetector {

public:
    EuclideanClusteringDetector(double cluster_tolerance, int min_cluster_size,
        int max_cluster_size, double range);

    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

private:
    double m_cluster_tolerance;
    int m_min_cluster_size;
    int m_max_cluster_size;
    double m_range;
};

#endif // EEUCLIDEAN_CLUSTERING_H