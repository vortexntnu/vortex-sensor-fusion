/**
 * @file dbscan_detector.hpp
 * @brief This file defines a DBSCAN detector that inherits from the IPclDetector interface.
 */

#pragma once

#include <pcl_detector/pcl_detector_interface.hpp>

#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcl_detector {

class DBSCANDetector : public IPclDetector {

public:
    DBSCANDetector(float eps, int min_points)
        : eps_(eps)
        , min_points_(min_points)
    {
    }

    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> get_clusters(const pcl::PointCloud<pcl::PointXYZ>& points) override;
    pcl::PointCloud<pcl::PointXYZ> get_centroids(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters) override;

private:
    float eps_;
    size_t min_points_;
};

}; // namespace pcl_detector