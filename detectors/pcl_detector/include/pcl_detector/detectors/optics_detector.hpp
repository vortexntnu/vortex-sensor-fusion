#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl_detector/pcl_detector.hpp"

namespace pcl_detector {

class OPTICSDetector : public IPclDetector {

public:
    OPTICSDetector(double eps, int min_points) : eps_(eps), min_points_(min_points) {}

    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

private:
    double eps_;
    int min_points_;
};

}; // namespace pcl_detector
