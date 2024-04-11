#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>

namespace pcl_detector {

class GeometryProcessor{
    

public:
    static void transformLines(std::vector<Eigen::VectorXf>& prev_lines,
                                       const Eigen::Vector3f& translation,
                                       const Eigen::Quaternionf& rotation);    

};

}; // namespace pcl_detector