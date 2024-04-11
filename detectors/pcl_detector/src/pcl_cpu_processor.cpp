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
                 float prev_line_thresh, float project_thresh, float wall_neighbour_dist, int wall_min_points, float wall_min_length, float wall_merge_dist)
    : voxel_leaf_size_(voxel_leaf_size), model_thresh_(model_thresh),
      model_iterations_(model_iterations), prev_line_thresh_(prev_line_thresh),
      project_thresh_(project_thresh), wall_neighbour_dist_(wall_neighbour_dist),
      wall_min_points_(wall_min_points), wall_min_length_(wall_min_length), wall_merge_dist_(wall_merge_dist) {}


void PclProcessor::applyPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& field_name, float limit_min, float limit_max)
{
    // no need to remove NaN points, as the pass through filter does it, also resizes cloud, sets height and width. isOrganized is set to false
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
    std::cout << "Downsample all data: " << voxel_grid.getDownsampleAllData() << std::endl;
    voxel_grid.setDownsampleAllData(false);
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


std::tuple<Eigen::VectorXf, std::vector<pcl::PointXYZ>> pcl_detector::PclProcessor::getWalls(const Eigen::VectorXf& coefficients, std::vector<int>& wall_indices, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    auto optimized_coefficients = optimizeLine(coefficients, inliers, cloud);
    auto [projected_cloud, inliers_projected] = projectInliersOnLine(optimized_coefficients, cloud);
    sortProjectedPoints(inliers_projected, projected_cloud);
    auto wall_poses = findWalls(projected_cloud, wall_indices); // findWalls now modifies wall_indices directly
    return {optimized_coefficients, wall_poses};
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
    std::cout << "Inliers size after optimization: " << inliers.size() << std::endl;

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


std::vector<pcl::PointXYZ> pcl_detector::PclProcessor::findWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud, std::vector<int>& wall_indices) {
    std::vector<pcl::PointXYZ> wall_poses; 

    int start_index = static_cast<int>(projectedCloud->points.front().z);
    std::vector<int> current_wall_indices = {start_index};
    pcl::PointXYZ start_point = projectedCloud->points.front(), end_point;

    for (size_t i = 1; i < projectedCloud->points.size(); ++i) {
        const auto& point = projectedCloud->points[i];
        const auto& prev_point = projectedCloud->points[i - 1];
        double distance = std::hypot(point.x - prev_point.x, point.y - prev_point.y);

        if (distance < wall_neighbour_dist_) {
            current_wall_indices.push_back(static_cast<int>(point.z));
            end_point = point;
            continue;
        } else {
            addOrMergeWall(wall_indices, wall_poses, current_wall_indices, start_point, end_point);
            current_wall_indices.clear();
            current_wall_indices.push_back(static_cast<int>(point.z));
            start_point = point;
            end_point = point;
        }
    }

    addOrMergeWall(wall_indices, wall_poses, current_wall_indices, start_point, end_point);

    return wall_poses;
}

void pcl_detector::PclProcessor::addOrMergeWall(
    std::vector<int>& wall_indices, 
    std::vector<pcl::PointXYZ>& wall_poses, 
    const std::vector<int>& current_wall_indices,
    const pcl::PointXYZ& start_point, 
    const pcl::PointXYZ& end_point) {

    if ((current_wall_indices.size() > u_int16_t(wall_min_points_)) 
        && std::hypot(start_point.x - end_point.x, start_point.y - end_point.y) > wall_min_length_){
        wall_indices.insert(wall_indices.end(), current_wall_indices.begin(), current_wall_indices.end());
        if (!wall_poses.empty() && std::hypot(start_point.x - wall_poses.back().x, start_point.y - wall_poses.back().y) < wall_merge_dist_) {
            // Merge with previous wall
            wall_poses.back() = end_point;
        } else {
            // Add as a new wall
            wall_poses.push_back(start_point);
            wall_poses.push_back(end_point);
        }
    }
}

void pcl_detector::PclProcessor::getPointsBehindWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<pcl::PointXYZ>& wall_poses, std::vector<int>& indices_to_remove){
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> polygons;
    polygons.reserve(wall_poses.size() / 2);
    for(size_t i = 0; i < wall_poses.size(); i+=2){
        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud = createPolygon(wall_poses[i], wall_poses[i+1]);
        polygons.push_back(polygon_cloud);
    }

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        for(size_t j = 0; j < polygons.size(); j++){
            if (isXYPointIn2DXYPolygon(point, *polygons[j])) {
                indices_to_remove.push_back(i);
                break;
            }
        }
    
        
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_detector::PclProcessor::createPolygon(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    // Calculate the magnitude of the vector from origin to point
        float length = 200.0;
        // Normalize the vector, then scale it to the new length

        pcl::PointXYZ p1_ext;
        pcl::PointXYZ p2_ext;
        float p1_magnitude = sqrt(p1.x * p1.x + p1.y * p1.y);
        float p2_magnitude = sqrt(p2.x * p2.x + p2.y * p2.y);
        p1_ext.x = (p1.x / p1_magnitude) * length;
        p1_ext.y = (p1.y / p1_magnitude) * length;
        p2_ext.x = (p2.x / p2_magnitude) * length;
        p2_ext.y = (p2.y / p2_magnitude) * length;
        p1_ext.z = p1.z;
        p2_ext.z = p2.z;

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        polygon_cloud->points.push_back(p1);
        polygon_cloud->points.push_back(p2);
        polygon_cloud->points.push_back(p2_ext);
        polygon_cloud->points.push_back(p1_ext);
        return polygon_cloud;
}

bool pcl_detector::PclProcessor::isXYPointIn2DXYPolygon (const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ> &polygon)
{
  bool in_poly = false;
  double x1, x2, y1, y2;

  const auto nr_poly_points = polygon.size ();
  // start with the last point to make the check last point<->first point the first one
  double xold = polygon[nr_poly_points - 1].x;
  double yold = polygon[nr_poly_points - 1].y;
  for (std::size_t i = 0; i < nr_poly_points; i++)
  {
    double xnew = polygon[i].x;
    double ynew = polygon[i].y;
    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return (in_poly);
}

void pcl_detector::PclProcessor::extractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& indices_to_remove)
{
    // Extract doesn't care about unique indices, but if the size of the indices_to_remove is greater than pcl size then it fails
    // Sort indices to remove duplicates
    if(indices_to_remove.size() >= cloud->size()){
    std::sort(indices_to_remove.begin(), indices_to_remove.end());

    // Unique removes consecutive duplicates and returns a new end iterator
    auto last = std::unique(indices_to_remove.begin(), indices_to_remove.end());

    // Erase the non-unique elements by specifying the new range
    indices_to_remove.erase(last, indices_to_remove.end());
    }

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices = indices_to_remove;


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(true); 
    extract.filter(*cloud); 

    // clear to use the same vector later
    indices_to_remove.clear();
}


}; // namespace pcl_detector