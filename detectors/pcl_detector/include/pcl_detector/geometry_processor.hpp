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
    static float distanceFromOriginToLine(const Eigen::VectorXf& line);
    static void sortLinesByProximityToOrigin(std::vector<Eigen::VectorXf>& lines);
    static bool isPointBehindWall(const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& Q);
    

};

}; // namespace pcl_detector