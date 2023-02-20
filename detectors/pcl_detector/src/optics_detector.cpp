#include "pcl_detector/detectors/optics_detector.hpp"



namespace pcl_detector {


pcl::PointCloud<pcl::PointXYZ> OPTICSDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override {
    pcl::PointCloud<pcl::PointXYZ> detections;

    // Create KD-Tree for efficient neighborhood search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(points.makeShared());

    std::vector<bool> visited(points.size(), false);
    std::vector<int> predecessor(points.size(), -1);
    std::vector<double> reachability_distance(points.size(), std::numeric_limits<double>::infinity());

    for (size_t i = 0; i < points.size(); ++i) {
        if (!visited[i]) {
            expand_cluster(points, kdtree, i, predecessor, visited, reachability_distance);
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

    for (size_t i = 0; i < points.size(); ++i) {
        if (reachability_distance[i] <= eps_) {
            int cluster_id = predecessor[i];
            if (cluster_id == -1) {
                continue;
            }

            while (clusters.size() <= static_cast<size_t>(cluster_id)) {
                clusters.emplace_back();
            }

            clusters[cluster_id].push_back(points[i]);
        }
    }

    for (const auto& cluster : clusters) {
        if (cluster.empty()) {
            continue;
        }

        pcl::PointXYZ centroid;
        pcl::computeCentroid(cluster, centroid);
        detections.push_back(centroid);
    }

    return detections;
}


void OPTICSDetector::expand_cluster(const pcl::PointCloud<pcl::PointXYZ>& points,
                    pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                    int point_idx,
                    std::vector<int>& predecessor,
                    std::vector<bool>& visited,
                    std::vector<double>& reachability_distance) {
    visited[point_idx] = true;

    std::vector<int> neighbors;
    std::vector<float> distances;
    kdtree.radiusSearch(point_idx, eps_, neighbors, distances);

    std::vector<double> core_distances(neighbors.size());
    for (size_t i = 0; i < neighbors.size(); ++i) {
        int neighbor_idx = neighbors[i];
        if (!visited[neighbor_idx]) {
            visited[neighbor_idx] = true;

            std::vector<int> neighbor_neighbors;
            std::vector<float> neighbor_distances;
            kdtree.radiusSearch(neighbor_idx, eps_, neighbor_neighbors, neighbor_distances);

            if (neighbor_neighbors.size() >= min_pts_) {
                std::vector<double> neighbor_core_distances(neighbor_neighbors.size());
                for (size_t j = 0; j < neighbor_neighbors.size(); ++j) {
                    neighbor_core_distances[j] = pcl::euclideanDistance(points[neighbor_idx], points[neighbor_neighbors[j]]);
                }
                std::sort(neighbor_core_distances.begin(), neighbor_core_distances.end());
                double core_distance = neighbor_core_distances[min_pts_ - 1];
                core_distances[i] = core_distance;
            } else {
                core_distances[i] = std::numeric_limits<double>::infinity();
            }
        } else {
            core_distances[i] = reachability_distance[neighbor_idx];
        }
    }

    for (size_t i = 0; i < neighbors.size(); ++i) {
        int neighbor_idx = neighbors[i];
        if (!visited[neighbor_idx]) {
            predecessor[neighbor_idx] = point_idx;
            reachability_distance[neighbor_idx] = std::max(core_distances[i], distances[i]);
        }
    }

    std::vector<size_t> sorted_neighbors(neighbors.size());
    std::iota(sorted_neighbors.begin(), sorted_neighbors.end(), 0);
    std::sort(sorted_neighbors.begin(), sorted_neighbors.end(), [&](size_t i, size_t j) {
        return core_distances[i] < core_distances[j] || (core_distances[i] == core_distances[j] && distances[i] < distances[j]);
    });

    for (size_t i : sorted_neighbors) {
        int neighbor_idx = neighbors[i];
        if (!visited[neighbor_idx]) {
            expand_cluster(points, kdtree, neighbor_idx, predecessor, visited, reachability_distance);
        }
    }
}

}; // namespace pcl_detector
