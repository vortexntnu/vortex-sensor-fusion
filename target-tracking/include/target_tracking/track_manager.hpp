#pragma once

#include <Eigen/Dense>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_filtering/filters/ipda.hpp>

using State4d = vortex::prob::Gauss4d;
using DynMod = vortex::models::ConstantVelocity<2>;
using SensorMod = vortex::models::IdentitySensorModel<4, 2>;
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;

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
    TrackManager();

    void updateTracks(std::vector<Eigen::Vector2d> measurements_, int update_interval, double confirmation_threshold, double gate_theshhold, double prob_of_detection, double prob_of_survival, double clutter_intensity);

    void deleteTracks(double deletion_threshold);

    void set_dyn_model(double std_velocity);

    void set_sensor_model(double std_measurement);

    std::vector<Track> getTracks() const { return tracks_; }

private:
    std::vector<Track> tracks_;

    // Dynamic model
    std::shared_ptr<DynMod> dyn_model_;

    // Sensor model
    std::shared_ptr<SensorMod> sensor_model_;

    // Tracker id
    int tracker_id_;
};