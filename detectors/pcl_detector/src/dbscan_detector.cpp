#include "pcl_detector/detectors/dbscan_detector.hpp"

#include <iostream>

namespace pcl_detector {

pcl::PointCloud<pcl::PointXYZ> DBSCANDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points)
{
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(points.makeShared());

    // Initialize point labels to be unclassified
    std::vector<int> labels(points.size(), -1);
    int label = 0;

    // Iterate through each point in the point cloud
    for (int i = 0; i < points.size(); ++i) {
        if (labels[i] != -1) {
            continue; // Already classified, skip to next point
        }

        std::vector<int> neighbors;
        std::vector<float> distances;
        tree->radiusSearch(points[i], m_eps, neighbors, distances); // Find neighbors within radius eps

        if (neighbors.size() < m_min_points) {
            labels[i] = 0; // Mark as noise
            continue;
        }

        label++; // Start a new cluster
        labels[i] = label;

        // Iterate through neighbors and expand cluster
        for (int j = 0; j < neighbors.size(); ++j) {
            int neighbor = neighbors[j];
            if (labels[neighbor] == 0) {
                labels[neighbor] = label; // Convert noise to border point
            } else if (labels[neighbor] != -1) {
                continue; // Already classified, skip to next neighbor
            }
            labels[neighbor] = label; // Add neighbor to current cluster
            std::vector<int> sub_neighbors;
            std::vector<float> sub_distances;
            tree->radiusSearch(points[neighbor], m_eps, sub_neighbors, sub_distances);
            if (sub_neighbors.size() >= m_min_points) {
                neighbors.insert(neighbors.end(), sub_neighbors.begin(), sub_neighbors.end());
            }
        }
    }

    // Extract the clusters from the point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    for (int i = 1; i <= label; ++i) {
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        for (int j = 0; j < labels.size(); ++j) {
            if (labels[j] == i) {
                indices->indices.push_back(j);
                labels[j] = -1; // Remove point from labels to avoid duplicates
            }
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(points.makeShared());
        extract.setIndices(indices);
        pcl::PointCloud<pcl::PointXYZ> cluster;
        extract.filter(cluster);
        if (cluster.size() >= m_min_points) {
            clusters.push_back(cluster);
        }
    }

    // Compute the centroids of each cluster
    pcl::PointCloud<pcl::PointXYZ> output;
    for (const auto& cluster : clusters) {
        pcl::PointXYZ centroid;
        pcl::computeCentroid(cluster, centroid);
        output.push_back(centroid);
    }
    return output;
}

}; // namespace pcl_detector