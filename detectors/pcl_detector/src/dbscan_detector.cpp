#include "pcl_detector/detectors/dbscan_detector.hpp"

namespace pcl_detector {


pcl::PointCloud<pcl::PointXYZ> DBSCANDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) {
    pcl::PointCloud<pcl::PointXYZ> detections;

    // create a FLANN k-d tree for efficient approximate nearest neighbor search
    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr tree(new pcl::search::FlannSearch<pcl::PointXYZ>);
    tree->setInputCloud(points.makeShared());

    // initialize labels
    std::vector<int> labels(points.size(), -1); // -1 for noise points, >= 0 for clustered points

    // initialize cluster ID and noise label
    int cluster_id = 0;
    const int noise_label = -1;

    // iterate through all points
    for (int i = 0; i < points.size(); i++) {
        if (labels[i] != -1) { // skip points that have already been assigned to a cluster
            continue;
        }

        std::vector<int> neighbors;
        std::vector<float> distances;

        // find all neighbors of this point within eps distance using FLANN
        tree->radiusSearch(points[i], eps_, neighbors, distances);

        if (neighbors.size() < min_points_) { // assign as noise if the point does not have enough neighbors
            labels[i] = noise_label;
            continue;
        }

        // assign a new cluster ID and add this point to it
        labels[i] = cluster_id;
        detections.push_back(points[i]);

        // expand the cluster by adding all reachable neighbors
        expand_cluster(points[i], cluster_id, neighbors, labels, detections, tree, eps_, min_points_);

        cluster_id++; // move to next cluster
    }

    return detections;
}

void DBSCANDetector::expand_cluster(const pcl::PointXYZ& point, int cluster_id, const std::vector<int>& neighbors,
                                    std::vector<int>& labels, pcl::PointCloud<pcl::PointXYZ>& detections,
                                    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree, float eps, int min_points) {
    // expand the cluster by adding all reachable neighbors
    for (int j = 0; j < neighbors.size(); j++) {
        if (labels[neighbors[j]] == noise_label) { // assign to cluster if neighbor was previously noise
            labels[neighbors[j]] = cluster_id;
            detections.push_back(points[neighbors[j]]);
        } else if (labels[neighbors[j]] == -1) { // mark as visited and add to neighbor list
            labels[neighbors[j]] = cluster_id;
            detections.push_back(points[neighbors[j]]);
            std::vector<int> more_neighbors;
            std::vector<float> more_distances;
            tree->radiusSearch(points[neighbors[j]], eps, more_neighbors, more_distances);
            if (more_neighbors.size() >= min_points) { // add new neighbors to neighbor list if they meet min_points requirement
                neighbors.insert(neighbors.end(), more_neighbors.begin(), more_neighbors.end());
            }
        }
    }
}

}; // namespace pcl_detector