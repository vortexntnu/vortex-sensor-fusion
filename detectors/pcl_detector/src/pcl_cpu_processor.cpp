#include <pcl_detector/pcl_processor.hpp>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>

#include <pcl_detector/sample_consensus/msac.h>
#include <pcl_detector/sample_consensus/sac_model_line_2d.hpp>
#include <algorithm> 
#include <vector>
#include <cmath>



namespace pcl_detector {

PclProcessor::PclProcessor(float voxel_leaf_size, float model_thresh, int model_iterations,
                 float prev_line_thresh, float project_thresh, float wall_neighbour_dist, int wall_min_points, float wall_min_length)
    : voxel_leaf_size_(voxel_leaf_size), model_thresh_(model_thresh),
      model_iterations_(model_iterations), prev_line_thresh_(prev_line_thresh),
      project_thresh_(project_thresh), wall_neighbour_dist_(wall_neighbour_dist),
      wall_min_points_(wall_min_points), wall_min_length_(wall_min_length) {}

void PclProcessor::remove_zero_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    cloud->erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
                                [](const pcl::PointXYZ& point) {
                                    return point.x == 0.0f && point.y == 0.0f && point.z == 0.0f;
                                }),
                 cloud->points.end());
}

void PclProcessor::applyPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& field_name, float limit_min, float limit_max)
{
    // no need to remove NaN points, as the pass through filter does it, also resizes cloud, sets height and width. isOrganized is set to false
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field_name);
    pass.setFilterLimits(limit_min, limit_max);
    pass.filter(*cloud);
}

void PclProcessor::apply_voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setDownsampleAllData(false);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.filter(*cloud);
}

std::tuple<Eigen::VectorXf, std::vector<int>> PclProcessor::find_line_with_MSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{
    // Define a model for the line
    pcl::SampleConsensusModelLine2D<pcl::PointXYZ>::Ptr 
        model_l(new pcl::SampleConsensusModelLine2D<pcl::PointXYZ>(cloud));

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

std::tuple<Eigen::VectorXf, std::vector<WallPose>, std::vector<std::vector<int>>> PclProcessor::get_walls(
const Eigen::VectorXf& coefficients, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    auto optimized_coefficients = optimize_line(coefficients, inliers, cloud);
    auto [projected_cloud, inliers_projected] = project_inliers_on_line(optimized_coefficients, cloud);
    auto distances_and_indices = sort_projected_points(inliers_projected, projected_cloud);
    auto [wall_poses, wall_indices] = find_walls_on_line(distances_and_indices, projected_cloud);
    return {optimized_coefficients, wall_poses, wall_indices};
}

Eigen::VectorXf PclProcessor::optimize_line(Eigen::VectorXf model, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::SampleConsensusModelLine2D<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine2D<pcl::PointXYZ>(cloud));
    Eigen::VectorXf optimized_coefficients;
    model_l->optimizeModelCoefficients(inliers, model, optimized_coefficients);
    return optimized_coefficients;
}

std::vector<int> PclProcessor::get_inliers(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::SampleConsensusModelLine2D<pcl::PointXYZ>::Ptr new_model_l(new pcl::SampleConsensusModelLine2D<pcl::PointXYZ>(cloud));

    std::vector<int> inliers;
    new_model_l->selectWithinDistance(coefficients, prev_line_thresh_, inliers);

    return inliers;
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>> PclProcessor::project_inliers_on_line(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::SampleConsensusModelLine2D<pcl::PointXYZ>::Ptr new_model_l(new pcl::SampleConsensusModelLine2D<pcl::PointXYZ>(cloud));

    std::vector<int> inliers;
    new_model_l->selectWithinDistance(coefficients, project_thresh_, inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    new_model_l->projectPoints(inliers, coefficients, *projectedCloud, false);

    return {projectedCloud, inliers};
}

std::vector<std::tuple<float, int, int>> PclProcessor::sort_projected_points(const std::vector<int>& inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud)
{
    if (inliers.empty())
    {
        PCL_ERROR("Inliers vector is empty.\n");
        return {};
    }

    // Variables to find the furthest point
    float maxDistance = -1.0f;
    int start_index = -1;

    // Find the furthest point on the line from the origin(0,0,0) in the projectedCloud
    // All other points will be sorted based on distance from this point
    for (size_t i = 0; i < inliers.size(); ++i)
    {
        auto& point = projectedCloud->points[i];
        float distance = point.x * point.x + point.y * point.y;
        
        if (distance > maxDistance)
        {
            maxDistance = distance;
            start_index = i;
        }
    }
    // Vector to store distances and original indices and indices in projected cloud
    std::vector<std::tuple<float, int, int>> distances_and_indices(inliers.size());

    const float x_0 = projectedCloud->points[start_index].x;
    const float y_0 = projectedCloud->points[start_index].y;
    // Calculate the squared distance of each point from the start point
    for (size_t i = 0; i < inliers.size(); ++i)
    {
        const auto& point = projectedCloud->points[i];
        const float distance = std::hypot(point.x - x_0, point.y - y_0);
        distances_and_indices[i] = std::make_tuple(distance, inliers.at(i), i);
    }

    // Sort the distances_and_indices vector based on the calculated distance
    std::sort(distances_and_indices.begin(), distances_and_indices.end(),
              [](const std::tuple<float, int, int>& a, const std::tuple<float, int, int>& b) -> bool {
                  return std::get<0>(a) < std::get<0>(b);
              });

    return distances_and_indices;
}

std::pair<std::vector<WallPose>, std::vector<WallIndices>> PclProcessor::find_walls_on_line(const std::vector<std::tuple<float, int, int>>& distances_and_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud)
{
    std::vector<WallPose> wall_poses;
    std::vector<WallIndices> wall_indices;
    // Get index of the first point in the original pointcloud
    int wall_start_index = std::get<1>(distances_and_indices.front());
    std::vector<int> current_wall_indices = {wall_start_index};
    pcl::PointXYZ wall_start_point = projectedCloud->points.at(std::get<2>(distances_and_indices.front()));
    pcl::PointXYZ wall_end_point;
    for (size_t i = 1; i < distances_and_indices.size(); ++i)
    {
        const float distance = std::get<0>(distances_and_indices[i])-std::get<0>(distances_and_indices[i-1]);
        if (distance < wall_neighbour_dist_)
        {
            current_wall_indices.push_back(std::get<1>(distances_and_indices[i]));
            wall_end_point = projectedCloud->points.at(std::get<2>(distances_and_indices[i]));
        }
        else if ((current_wall_indices.size() > std::size_t(wall_min_points_))
            && (std::hypot(wall_start_point.x - wall_end_point.x, wall_start_point.y - wall_end_point.y) > wall_min_length_))
        {
            // Add new wall and indices
            wall_indices.push_back(current_wall_indices);
            wall_poses.push_back({wall_start_point, wall_end_point});

            // Setup for further detection
            current_wall_indices.clear();
            current_wall_indices.push_back(std::get<1>(distances_and_indices[i]));
            wall_start_point = projectedCloud->points.at(std::get<2>(distances_and_indices[i]));
            wall_end_point = wall_start_point;
        } else {
            current_wall_indices.clear();
            current_wall_indices.push_back(std::get<1>(distances_and_indices[i]));
            wall_start_point = projectedCloud->points.at(std::get<2>(distances_and_indices[i]));
            wall_end_point = wall_start_point;
        }
    }
    // Check if current sample is eligible to be a wall
    if ((current_wall_indices.size() > u_int16_t(wall_min_points_)) 
        && std::hypot(wall_start_point.x - wall_end_point.x, wall_start_point.y - wall_end_point.y) > wall_min_length_)
    {
        wall_indices.push_back(current_wall_indices);
        wall_poses.push_back({wall_start_point, wall_end_point});
    }

    return {wall_poses, wall_indices};
}

void PclProcessor::get_points_behind_walls(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<WallPose>& wall_poses, std::vector<int>& indices_to_remove)
{
    struct WallPolygons{
        int nvert;
        float* vertx;
        float* verty;
    };

    std::vector<WallPolygons> wall_polygons;
    wall_polygons.reserve(wall_poses.size());
    
    for(size_t i = 0; i < wall_poses.size(); i++)
    {
        auto [first_e_x,first_e_y] = create_extended_point(wall_poses[i].first);
        auto [second_e_x,second_e_y] = create_extended_point(wall_poses[i].second);
        wall_polygons.push_back({4, new float[4]{wall_poses[i].first.x, wall_poses[i].second.x, second_e_x, first_e_x},
                                    new float[4]{wall_poses[i].first.y, wall_poses[i].second.y, second_e_y, first_e_y}});
    }

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const auto& point = cloud->points[i];
        const auto& p_x = point.x;
        const auto& p_y = point.y;
        for(size_t j = 0; j < wall_polygons.size(); j++)
        {
            if (pnpoly(wall_polygons[j].nvert, wall_polygons[j].vertx, wall_polygons[j].verty, p_x, p_y))
            {
                indices_to_remove.push_back(i);
                break;
            }
        }   
    }
}

std::pair<float,float> PclProcessor::create_extended_point(const pcl::PointXYZ& p)
{
    // Calculate the magnitude of the vector from origin to point
        float length = 200.0;
        // Normalize the vector, then scale it to the new length

        pcl::PointXYZ p1_ext;
        float p_magnitude = sqrt(p.x * p.x + p.y * p.y);
        float e_x = (p.x / p_magnitude) * length;
        float e_y = (p.y / p_magnitude) * length;

        return {e_x, e_y};
}

int PclProcessor::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

void PclProcessor::filter_lines(std::vector<LineData>& lines_data)
{   
    struct WallPolygons{
        int nvert;
        float* vertx;
        float* verty;
    };

    std::vector<WallPolygons> wall_polygons;
    
    // Create polygons for walls on each line
    for (const auto& line_data : lines_data)
    {
        for (const auto& wall : line_data.wall_poses)
        {
        auto [first_e_x,first_e_y] = create_extended_point(wall.first);
        auto [second_e_x,second_e_y] = create_extended_point(wall.second);
        wall_polygons.push_back({4, new float[4]{wall.first.x, wall.second.x, second_e_x, first_e_x},
                                    new float[4]{wall.first.y, wall.second.y, second_e_y, first_e_y}});
        }
    }

    // Check if the centre of walls are inside the polygons formed by other walls
    size_t wall_number = 0;
    for (size_t i = 0; i < lines_data.size(); ++i)
    {
        for (size_t j = 0; j < lines_data[i].wall_poses.size(); ++j)
        {
            float wall_center_x = (lines_data[i].wall_poses[j].first.x + lines_data[i].wall_poses[j].second.x) / 2;
            float wall_center_y = (lines_data[i].wall_poses[j].first.y + lines_data[i].wall_poses[j].second.y) / 2;
            
            // Check if the current wall is inside any of the polygons formed by other walls
            for (size_t k = 0; k < wall_polygons.size(); ++k)
            {
                if(k == wall_number) continue; // Skip the same wall (polygon)
                if (pnpoly(wall_polygons[k].nvert, wall_polygons[k].vertx, wall_polygons[k].verty, wall_center_x, wall_center_y))
                {
                    // Mark the wall as inactive
                    lines_data[i].walls_active[j] = false;
                    break;
                }
            }
            wall_number++;
        }
    }
    
    for (auto& line_data : lines_data) {
        std::vector<WallPose> active_wall_poses; // To store poses of active walls
        std::vector<WallIndices> active_wall_indices; // To store indices of active

        // Process each wall in the current line
        for (size_t j = 0; j < line_data.wall_poses.size(); ++j) {
            if (!line_data.walls_active[j]) continue; // Skip walls marked as inactive

            // For active walls, add both wall pose and indices to the active vectors
            active_wall_poses.push_back(line_data.wall_poses[j]);
            active_wall_indices.push_back(line_data.wall_indices[j]);
        }

        // Update the line's wall poses and indices with only the active walls
        line_data.wall_poses = std::move(active_wall_poses);
        line_data.wall_indices = std::move(active_wall_indices);
    }

    // Remove lines from lines_data that have no walls left
    lines_data.erase(std::remove_if(lines_data.begin(), lines_data.end(), 
                                   [](const LineData& ld) { return ld.wall_poses.empty(); }),
                    lines_data.end());
}

void PclProcessor::apply_landmask(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int nvert, float* vertx, float* verty)
{
    std::vector<int> indices_to_remove;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        // Using pnpoly to check if the point is within the land_mask polygon
        if (!pnpoly(nvert, vertx, verty, point.x, point.y)) {
            indices_to_remove.push_back(i);
        }
    }

    // Function to remove points based on indices_to_remove
    extract_points(cloud, indices_to_remove);
}

void PclProcessor::extract_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& indices_to_remove)
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

void PclProcessor::compute_convex_hull(const pcl::PointCloud<pcl::PointXYZ>& cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr& hull)
{
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setDimension(2);
    chull.setInputCloud(cluster.makeShared());
    chull.reconstruct(*hull);
}

}; // namespace pcl_detector