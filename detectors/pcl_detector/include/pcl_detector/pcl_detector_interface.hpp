#ifndef PCL_DETECTOR_H
#define PCL_DETECTOR_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl_detector {


/**
 * @brief A interface for pcl detectors
 */
class IPclDetector {

public:
    virtual ~IPclDetector() = default;

    virtual pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) = 0;
    virtual std::vector<pcl::PointCloud<pcl::PointXYZ>> get_clusters(const pcl::PointCloud<pcl::PointXYZ>& points) = 0;
    virtual pcl::PointCloud<pcl::PointXYZ> get_centroids(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters) = 0;
};

}; // namespace pcl_detector

#endif // PCL_DETECTOR_H