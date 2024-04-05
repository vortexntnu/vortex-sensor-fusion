#include <pcl_detector/geometry_processor.hpp>
namespace pcl_detector {

std::vector<Eigen::VectorXf> GeometryProcessor::transformLines(const std::vector<Eigen::VectorXf>& prev_lines,
                                                             const Eigen::Vector3f& translation,
                                                             const Eigen::Quaternionf& rotation) 
{
    std::vector<Eigen::VectorXf> transformed_lines;

    for (const auto& line : prev_lines) {
        Eigen::VectorXf transformed_line(6); // Ensure your line is sized appropriately.

        // Extract the point and direction vector from the line
        Eigen::Vector3f point(line[0], line[1], line[2]);
        Eigen::Vector3f direction(line[3], line[4], line[5]);

        // Apply the rotation
        Eigen::Vector3f rotated_point = rotation * point;
        Eigen::Vector3f rotated_direction = rotation * direction;

        // Apply the translation to the point
        Eigen::Vector3f translated_point = rotated_point + translation;

        // Reassemble the transformed line
        transformed_line << translated_point[0], translated_point[1], translated_point[2],
                            rotated_direction[0], rotated_direction[1], rotated_direction[2];

        transformed_lines.push_back(transformed_line);
    }

    return transformed_lines;
}

float GeometryProcessor::distanceFromOriginToLine(const Eigen::VectorXf& line) 
{
    Eigen::Vector3f point(line[0], line[1], line[2]); // Point on the line
    Eigen::Vector3f direction(line[3], line[4], line[5]); // Direction vector

    // Vector from origin to the point on the line is just the point itself
    // Cross product between (point) and (direction)
    Eigen::Vector3f cross_product = point.cross(direction);

    // Distance formula
    return cross_product.norm() / direction.norm();
}

void GeometryProcessor::sortLinesByProximityToOrigin(std::vector<Eigen::VectorXf>& lines)
{
    std::sort(lines.begin(), lines.end(), [](const Eigen::VectorXf& a, const Eigen::VectorXf& b) {
        return distanceFromOriginToLine(a) < distanceFromOriginToLine(b);
    });
}

// Check if the intersection occurs closer to Q than to the origin, from the perspective of Q.
bool GeometryProcessor::isPointBehindWall(const Eigen::Vector2f& P1, const Eigen::Vector2f& P2, const Eigen::Vector2f& Q)
{
    Eigen::Vector2f direction = P2 - P1; // Direction of the line segment P1-P2
    Eigen::Vector2f P1_to_Q = Q - P1;

    // Coefficients for the linear equations
    float a = direction[0];
    float b = -P1_to_Q[0];
    float c = direction[1];
    float d = -P1_to_Q[1];

    // Solve for t (position on P1-P2) and s (position on Q-O, negative towards O)
    float det = a * d - b * c;
    if (std::abs(det) < 1e-6) return false; // Lines are parallel or coincident

    float t = (d * P1_to_Q.x() - b * P1_to_Q.y()) / det;
    if(!(t >= 0.0f && t <= 1.0f)) return false; // Intersection point is outside the line segment

    // Intersection point on P1-P2
    Eigen::Vector2f intersection = P1 + t * direction;

    // Check if intersection is closer to Q than to the origin
    float dist_to_Q = (intersection - Q).squaredNorm();
    float dist_to_O = Q.squaredNorm(); // Since origin is (0,0), direct distance to Q

    return dist_to_Q < dist_to_O;
}

}; // namespace pcl_detector