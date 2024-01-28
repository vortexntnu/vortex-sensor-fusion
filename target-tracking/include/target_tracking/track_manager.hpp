#pragma once

#include <Eigen/Dense>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_filtering/filters/pdaf.hpp>

using State4d = vortex::prob::Gauss4d;
using DynMod = vortex::models::ConstantVelocity<2>;
using SensorMod = vortex::models::IdentitySensorModel<4, 2>;
using PDAF = vortex::filter::PDAF<vortex::models::ConstantVelocity<2>, vortex::models::IdentitySensorModel<4, 2>>;

struct Track {
    int id;
    State4d state;
    double existence_probability;
    bool confirmed;

    // For sorting tracks based on existence probability and confirmed track
    bool operator<(const Track &other) const {
        if (confirmed != other.confirmed) {
            return confirmed > other.confirmed;
        } else {
            return existence_probability < other.existence_probability;
        }
    }

    bool operator==(const Track &other) const {
        return confirmed == other.confirmed && existence_probability == other.existence_probability;
    }
};

class TrackManager {
public:
    TrackManager(double std_velocity, double std_sensor);

    TrackManager() = default;

    void updateTracks(std::vector<Eigen::Vector2d> measurements_, int update_interval, double confirmation_threshold, double gate_theshhold, double prob_of_detection, double clutter_intensity);

    void deleteTracks(double deletion_threshold);

    std::vector<Track> getTracks() const { return tracks_; }

private:
    std::vector<Track> tracks_;

    // Dynamic model
    std::shared_ptr<DynMod> dyn_model_;

    // Sensor model
    std::shared_ptr<SensorMod> sensor_model_;

    // PDAF filter
    std::shared_ptr<PDAF> pdaf_;

    // Tracker id
    int tracker_id_;
};