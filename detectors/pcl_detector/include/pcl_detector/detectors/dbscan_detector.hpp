/**
 * @file dbscan_detector.hpp
 * @brief This file defines a DBSCAN detector that inherits from the IPclDetector interface.
 */

#ifndef DBSCAN_DETECTOR_H
#define DBSCAN_DETECTOR_H

#include <pcl/segmentation/extract_clusters.h>

#include "pcl_detector/pcl_detector.hpp"

namespace pcl_detector {

/**
 * @class DBSCANDetector
 * @brief A DBSCAN detector that implements the IPclDetector interface.
 */
class DBSCANDetector : public IPclDetector {

public:
    /**
     * @brief Constructs a DBSCAN detector with the given parameters.
     * @param eps The maximum distance between two points for them to be considered as part of the same cluster.
     * @param min_points The minimum number of points for a cluster to be considered valid.
     */
    DBSCANDetector(double eps, int min_points) : eps_(eps), min_points_(min_points) {}

    /**
     * @brief Detects clusters in the given point cloud using DBSCAN.
     * @param points The input point cloud.
     * @return The detected clusters as a point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

private:
    double eps_; /**< The maximum distance between two points for them to be considered as part of the same cluster. */
    int min_points_; /**< The minimum number of points for a cluster to be considered valid. */

};

}; // namespace pcl_detector

#endif // DBSCAN_DETECTOR_H
