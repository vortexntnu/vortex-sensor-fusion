/**
 * @file dbscan_detector.hpp
 * @brief This file defines a DBSCAN detector that inherits from the IPclDetector interface.
 */

#ifndef DBSCAN_DETECTOR_H
#define DBSCAN_DETECTOR_H

#include "pcl_detector/pcl_detector.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>


namespace pcl_detector {

class DBSCANDetector : public IPclDetector {

public:
    DBSCANDetector(float eps, int min_points) : m_eps(eps), m_min_points(min_points) {}

    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

private:
    float m_eps;
    int m_min_points;
};

}; // namespace pcl_detector

#endif // DBSCAN_DETECTOR_H
