#pragma once

#include <pcl_detector/pcl_detector_interface.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcl_detector {

class EuclideanClusteringDetector : public IPclDetector {

public:
    EuclideanClusteringDetector(double cluster_tolerance, int min_cluster_size);

    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> get_clusters(const pcl::PointCloud<pcl::PointXYZ>& points) override;
    pcl::PointCloud<pcl::PointXYZ> get_centroids(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters) override;

private:
    double cluster_tolerance_;
    int min_cluster_size_;
};

}; // namespace pcl_detector