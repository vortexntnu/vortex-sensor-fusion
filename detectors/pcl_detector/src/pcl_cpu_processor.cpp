#include <pcl_detector/pcl_processor.hpp>
#include <pcl_detector/geometry_processor.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <algorithm> 
#include <vector>
#include <cmath>



namespace pcl_detector {

PclProcessor::PclProcessor(float voxel_leaf_size, float model_thresh, int model_iterations,
                 float prev_line_thresh, float project_thresh, float wall_min_dist, int wall_min_points)
    : voxel_leaf_size_(voxel_leaf_size), model_thresh_(model_thresh),
      model_iterations_(model_iterations), prev_line_thresh_(prev_line_thresh),
      project_thresh_(project_thresh), wall_min_dist_(wall_min_dist),
      wall_min_points_(wall_min_points) {}

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

void pcl_detector::PclProcessor::applyVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

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
    msac.setDistanceThreshold(model_thresh_);
    msac.setMaxIterations(model_iterations_);

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
    new_model_l->selectWithinDistance(coefficients, prev_line_thresh_, inliers);

    return inliers;
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>> pcl_detector::PclProcessor::projectInliersOnLine(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr new_model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));

    std::vector<int> inliers;
    new_model_l->selectWithinDistance(coefficients, project_thresh_, inliers);
    std::cout << "Inliers size: " << inliers.size() << std::endl;

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
    // Minimum distance between points to be considered part of the same wall. 1.5m was optimal ish for Brattørkaia, became to cluttered for Pirbadet. 0.5 was better there
    int start_index = static_cast<int>(projectedCloud->points.at(0).z);
    std::vector<int> wall = {start_index};
    pcl::PointXYZ start_point = projectedCloud->points.at(0);

    for (uint16_t i = 1; i < projectedCloud->points.size()-1; i++)
    {
        auto& point = projectedCloud->points[i];
        auto& prev_point = projectedCloud->points[i - 1];
        double distance = std::sqrt(std::pow(point.x - prev_point.x, 2) + std::pow(point.y - prev_point.y, 2));
        if (distance < wall_min_dist_)
        {
            wall.push_back(static_cast<int>(projectedCloud->points.at(i).z));
        }
        else
        {
            if (wall.size() > u_int16_t(wall_min_points_))
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
    if(wall.size() > u_int16_t(wall_min_points_))
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

// pcl::PointIndices::Ptr pcl_detector::PclProcessor::getPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& P1, const Eigen::Vector3f& P2)
// {
//     Eigen::Vector2f P1_2D(P1[0], P1[1]), P2_2D(P2[0], P2[1]);

//     pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());

//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//         const auto& point = cloud->points[i];
//         Eigen::Vector2f Q(point.x, point.y);

//         // If the point meets the criteria for removal, add its index
//         if (GeometryProcessor::isPointBehindWall(P1_2D, P2_2D, Q)) {
//             indices_to_remove->indices.push_back(i);
//         }
//     }
//     return indices_to_remove;
// }
pcl::PointIndices::Ptr pcl_detector::PclProcessor::getPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ& P1, const pcl::PointXYZ& P2) 
{

    // Calculate the magnitude of the vector from origin to point
    float length = 200.0;
    // Normalize the vector, then scale it to the new length
    
    pcl::PointXYZ P1_ext;
    pcl::PointXYZ P2_ext;
    float P1_magnitude = sqrt(P1.x * P1.x + P1.y * P1.y + P1.z * P1.z);
    float P2_magnitude = sqrt(P2.x * P2.x + P2.y * P2.y + P2.z * P2.z);
    P1_ext.x = (P1.x / P1_magnitude) * length;
    P1_ext.y = (P1.y / P1_magnitude) * length;
    P2_ext.x = (P2.x / P2_magnitude) * length;
    P2_ext.y = (P2.y / P2_magnitude) * length;

    // Calculate extended points from P1 and P2 out to a distance of 100 units


    pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    polygon_cloud->points.push_back(P1);
    polygon_cloud->points.push_back(P2);
    polygon_cloud->points.push_back(P2_ext);
    polygon_cloud->points.push_back(P1_ext);
   

    float max_x = std::numeric_limits<float>::lowest(); // Start with the lowest possible value
    pcl::PointXYZ max_x_point;

    pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
         if (point.x > max_x) {
            max_x = point.x;
            max_x_point = point;
        }
        
        // Now isXYPointIn2DXYPolygon expects pcl::PointXYZ point and pcl::PointCloud<pcl::PointXYZ>
        if (pcl::isXYPointIn2DXYPolygon<pcl::PointXYZ>(point, *polygon_cloud)) {
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