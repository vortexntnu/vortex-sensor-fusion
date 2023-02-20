#include "pcl_detector/detectors/optics_detector.hpp"



namespace pcl_detector {


 pcl::PointCloud<pcl::PointXYZ> OPTICSDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) {
    // Convert the input point cloud to a matrix of point coordinates
    PointMatrix point_matrix(points.size(), std::vector<double>(3));
    for (size_t i = 0; i < points.size(); i++) {
        point_matrix[i][0] = points[i].x;
        point_matrix[i][1] = points[i].y;
        point_matrix[i][2] = points[i].z;
    }

    // Create a KdTree for fast nearest neighbor searches
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(points.makeShared());

    // Compute the OPTICS reachability and core distance for each point
    std::vector<double> core_distances(points.size());
    std::vector<double> reachability_distances(points.size());
    std::vector<int> predecessors(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        // Find the indices and distances of the nearest neighbors within the epsilon neighborhood
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        kd_tree->radiusSearch(points[i], eps_, nn_indices, nn_distances);

        // If the point has too few neighbors, mark it as noise
        if (nn_indices.size() < min_points_) {
            reachability_distances[i] = std::numeric_limits<double>::infinity();
            core_distances[i] = std::numeric_limits<double>::infinity();
            predecessors[i] = -1;
        } else {
            // Compute the core distance as the distance to the k-th nearest neighbor
            std::sort(nn_distances.begin(), nn_distances.end());
            double core_distance = nn_distances[min_points_ - 1];
            core_distances[i] = core_distance;

            // Compute the reachability distance as the maximum of the core distance and the distance to the predecessor
            double reachability_distance = std::max(core_distance, 0.0);
            int predecessor = -1;
            for (int j : nn_indices) {
                if (reachability_distances[j] < reachability_distance) {
                    reachability_distance = reachability_distances[j];
                    predecessor = j;
                }
            }
            reachability_distances[i] = reachability_distance;
            predecessors[i] = predecessor;
        }
    }

    // Sort the points by reachability distance to obtain a reachability plot
    std::vector<int> sorted_indices(points.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&reachability_distances](int i, int j) { return reachability_distances[i] < reachability_distances[j]; });

    // Perform hierarchical clustering on the reachability plot to extract the clusters
    std::vector<int> cluster_labels(points.size(), -1);
    int current_label = 0;
    for (int idx : sorted_indices) {
        if (cluster_labels[idx] != -1 || core_distances[idx] == std::numeric_limits<double>::infinity()) {
            // Skip points that were already assigned to a cluster or are noise
            continue;
            }


        // Start a new cluster with the current point as the core point
        std::vector<int> cluster_indices;
        cluster_indices.push_back(idx);
        cluster_labels[idx] = current_label;

        // Expand the cluster by adding all density-reachable points
        int current_point = idx;
        while (predecessors[current_point] != -1) {
            current_point = predecessors[current_point];
            if (cluster_labels[current_point] == -1) {
                cluster_indices.push_back(current_point);
                cluster_labels[current_point] = current_label;
            }
        }

        // If the cluster is too small, mark all points as noise instead
        if (cluster_indices.size() < min_points_) {
            for (int i : cluster_indices) {
                cluster_labels[i] = -1;
            }
        } else {
            current_label++;
        }
    }

    // Extract the centroids of the clusters as the detections
    pcl::PointCloud<pcl::PointXYZ> detections;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setClusterTolerance(eps_);
    cluster_extraction.setMinClusterSize(min_points_);
    cluster_extraction.setMaxClusterSize(points.size());
    cluster_extraction.setSearchMethod(kd_tree);
    cluster_extraction.setInputCloud(points.makeShared());
    cluster_extraction.setIndices(pcl::PointIndices::Ptr(new pcl::PointIndices));
    cluster_extraction.extract(detections_indices_);
    detections.width = detections_indices_->indices.size();
    detections.height = 1;
    detections.points.resize(detections.width);
    for (size_t i = 0; i < detections_indices_->indices.size(); i++) {
        detections.points[i] = points.points[detections_indices_->indices[i]];
    }

    return detections;
}

}; // namespace pcl_detector
