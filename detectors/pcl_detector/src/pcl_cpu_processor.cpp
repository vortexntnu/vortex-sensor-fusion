#include <pcl_detector/pcl_processor.hpp>
#include <pcl_detector/geometry_processor.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <algorithm> 
#include <vector>
#include <cmath>



namespace pcl_detector {

std::vector<int> PclProcessor::removeNanPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    return indices;
}

void PclProcessor::applyPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& field_name, float limit_min, float limit_max)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field_name);
    pass.setFilterLimits(limit_min, limit_max);
    pass.filter(*cloud);
}

void PclProcessor::flattenPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float new_z_value)
{
    for (auto& point : cloud) {
        point.z = new_z_value;
    }
}

void pcl_detector::PclProcessor::applyVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

    // Use the same point cloud object for the output
    voxel_grid.filter(*cloud);
}


std::tuple<Eigen::VectorXf, std::vector<int>> pcl_detector::PclProcessor::findLineWithMSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{
    // Define a model for the line
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr 
        model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));

    // Create the MSAC object
    pcl::MEstimatorSampleConsensus<pcl::PointXYZ> msac(model_l);
    msac.setDistanceThreshold(0.5);
    msac.setMaxIterations(100);

    Eigen::VectorXf coefficients;
    std::vector<int> inliers;

    msac.computeModel();
    msac.getInliers(inliers);
    msac.getModelCoefficients(coefficients);

    if (inliers.size() == 0) {
        PCL_ERROR("Could not estimate a line model for the given dataset.");
    }
    return {coefficients, inliers};
}

std::tuple<Eigen::VectorXf,std::vector<std::vector<int>>, std::vector<pcl::PointXYZ>> pcl_detector::PclProcessor::handleLine(const Eigen::VectorXf& coefficients, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    auto optimized_coefficients = optimizeLine(coefficients, inliers, cloud);
    auto [projected_cloud, inliers_projected] = projectInliersOnLine(optimized_coefficients, cloud);
    sortProjectedPoints(inliers_projected, projected_cloud);
    auto [wall_indices, wall_poses] = findWalls(projected_cloud);
    return {optimized_coefficients, wall_indices, wall_poses};
}

Eigen::VectorXf pcl_detector::PclProcessor::optimizeLine(Eigen::VectorXf model, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    Eigen::VectorXf optimized_coefficients;
    model_l->optimizeModelCoefficients(inliers, model, optimized_coefficients);
    return optimized_coefficients;
}

std::vector<int> pcl_detector::PclProcessor::findInliers(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr new_model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));

    std::vector<int> inliers;
    new_model_l->selectWithinDistance(coefficients, 0.5, inliers);

    return inliers;
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>> pcl_detector::PclProcessor::projectInliersOnLine(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr new_model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));

    std::vector<int> inliers;
    new_model_l->selectWithinDistance(coefficients, 0.5, inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    new_model_l->projectPoints(inliers, coefficients, *projectedCloud, false);

    return {projectedCloud, inliers};
}


void pcl_detector::PclProcessor::sortProjectedPoints(const std::vector<int>& inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud)
{
    if (inliers.empty())
    {
        PCL_ERROR("Inliers vector is empty.\n");
        return;
    }

    // Variables to find the furthest point
    float maxDistance = -1.0f;
    pcl::PointXYZ furthestPoint;

    // Find the furthest point from the origin in the copied projectedCloud cloud
    // Also sets the z-value of each point in the projectedCloud to its original index in cloud
        for (uint16_t i = 0; i < inliers.size(); i++) 
    {
        auto& point = projectedCloud->points[i];
        point.z = static_cast<float>(inliers.at(i));
        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            furthestPoint = point;
        }
    }

    // Sort the projectedCloud copy based on distance to the furthest point
    std::sort(projectedCloud->points.begin(), projectedCloud->points.end(),
              [&furthestPoint](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) -> bool {
                  float dist1 = std::pow(p1.x - furthestPoint.x, 2) + std::pow(p1.y - furthestPoint.y, 2);
                  float dist2 = std::pow(p2.x - furthestPoint.x, 2) + std::pow(p2.y - furthestPoint.y, 2);
                  return dist1 < dist2;
              });

}

std::tuple<std::vector<std::vector<int>>, std::vector<pcl::PointXYZ>> pcl_detector::PclProcessor::findWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud)
{
    std::vector<std::vector<int>> wall_indices;
    std::vector<pcl::PointXYZ> wall_poses;
    uint16_t wall_threshold = 50;
    double wall_distance = 1.5;
    int start_index = static_cast<int>(projectedCloud->points.at(0).z);
    std::vector<int> wall = {start_index};
    pcl::PointXYZ start_point = projectedCloud->points.at(0);

    for (uint16_t i = 1; i < projectedCloud->points.size()-1; i++)
    {
        auto& point = projectedCloud->points[i];
        auto& prev_point = projectedCloud->points[i - 1];
        double distance = std::sqrt(std::pow(point.x - prev_point.x, 2) + std::pow(point.y - prev_point.y, 2));
        if (distance < wall_distance)
        {
            wall.push_back(static_cast<int>(projectedCloud->points.at(i).z));
        }
        else
        {
            if (wall.size() > wall_threshold)
            {
                wall_indices.push_back(wall);
                wall_poses.push_back(start_point);
                wall_poses.push_back(prev_point);
            }
            wall.clear();
            wall.push_back(static_cast<int>(projectedCloud->points.at(i).z));
            start_point = point;
        }
    }
    if(wall.size() > wall_threshold)
    {
        wall_indices.push_back(wall);
        wall_poses.push_back(start_point);
        wall_poses.push_back(projectedCloud->points.back());
    }
    return {wall_indices, wall_poses};
}

void pcl_detector::PclProcessor::extractWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<std::vector<int>> wall_indices)
{
    // Accumulate all indices from each wall into a single set to ensure uniqueness
    std::set<int> all_indices_to_remove;
    for (const auto& wall : wall_indices) {
        all_indices_to_remove.insert(wall.begin(), wall.end());
    }

    // Convert the set of all indices to remove into a PointIndices::Ptr
    pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices);
    indices_to_remove->indices.assign(all_indices_to_remove.begin(), all_indices_to_remove.end());

    // Set up the extractor once with the accumulated indices
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices_to_remove);
    extract.setNegative(true);
    extract.filter(*cloud);
}

pcl::PointIndices::Ptr pcl_detector::PclProcessor::getPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& P1, const Eigen::Vector3f& P2)
{
    Eigen::Vector2f P1_2D(P1[0], P1[1]), P2_2D(P2[0], P2[1]);

    pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        Eigen::Vector2f Q(point.x, point.y);

        // If the point meets the criteria for removal, add its index
        if (GeometryProcessor::isPointBehindWall(P1_2D, P2_2D, Q)) {
            indices_to_remove->indices.push_back(i);
        }
    }
    return indices_to_remove;
}

void pcl_detector::PclProcessor::extractPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices_to_remove)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices_to_remove);
    extract.setNegative(true); 
    extract.filter(*cloud); 
}


}; // namespace pcl_detector