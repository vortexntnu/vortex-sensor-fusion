#include "pcl_detector/pcl_detector.hpp"

#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <numeric>

namespace pcl_detector {

class OPTICSDetector : public IPclDetector {
public:
    OPTICSDetector(double eps, int min_pts) : eps_(eps), min_pts_(min_pts) {}

    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;
private:


void expand_cluster(const pcl::PointCloud<pcl::PointXYZ>& points,
                    pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                    int point_idx,
                    std::vector<int>& predecessor,
                    std::vector<bool>& visited,
                    std::vector<float>& reachability_distance);

    double eps_;
    int min_pts_;

};

}; // namespace pcl_detector



