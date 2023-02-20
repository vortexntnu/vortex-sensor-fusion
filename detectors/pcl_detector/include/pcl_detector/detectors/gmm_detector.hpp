#ifndef GMM_DETECTOR_H
#define GMM_DETECTOR_H

#include <vector>

#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_detector/pcl_detector.hpp"

namespace pcl_detector {

class GMMDetector : public IPclDetector {
public:
    GMMDetector(int num_clusters, int max_iterations);

    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

private:
    int num_clusters_;
    int max_iterations_;
};

} // namespace pcl_detector

#endif // GMM_DETECTOR_H
