#include <pcl_detector/geometry_processor.hpp>
namespace pcl_detector {

void GeometryProcessor::transformLines(std::vector<Eigen::VectorXf>& prev_lines,
                                       const Eigen::Vector3f& translation,
                                       const Eigen::Quaternionf& rotation) {
    for (auto& line : prev_lines) { 
        // Assuming line is already of size 6
        Eigen::Vector3f point(line[0], line[1], line[2]);
        Eigen::Vector3f direction(line[3], line[4], line[5]);

        // Apply the rotation
        Eigen::Vector3f rotated_point = rotation * point;
        Eigen::Vector3f rotated_direction = rotation * direction;

        // Apply the translation to the point
        Eigen::Vector3f translated_point = rotated_point + translation;

        line[0] = translated_point[0];
        line[1] = translated_point[1];
        line[2] = translated_point[2];
        line[3] = rotated_direction[0];
        line[4] = rotated_direction[1];
        line[5] = rotated_direction[2];
    }
}

}; // namespace pcl_detector