#pragma once

#include <Eigen/Dense>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_filtering/filters/ipda.hpp>

using State4d = vortex::prob::Gauss4d;
using State2d = vortex::prob::Gauss<2>;
using DynMod = vortex::models::ConstantVelocity;
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

    /**
     * @brief Default constructor for the TrackManager class.
     */
    TrackManager();

    /**
     * @brief Updates the tracks based on the measurements received from the sensor.
     * 
     * @param measurements_ The measurements received from the sensor.
     * @param update_interval The time interval between updates.
     * @param confirmation_threshold The threshold for confirming a track.
     * @param gate_theshhold The threshold for gating measurements.
     * @param prob_of_detection The probability of detection.
     * @param prob_of_survival The probability of survival.
     * @param clutter_intensity The intensity of clutter.
     */
    void updateTracks(std::vector<Eigen::Vector2d> measurements_, int update_interval, double confirmation_threshold, double gate_theshhold, double min_gate_threshold, double max_gate_threshold, double prob_of_detection, double prob_of_survival, double clutter_intensity);

    /**
     * @brief Creates new tracks for every measurements.
     * 
     * @param measurements The measurements received.
     */
    void createTracks(std::vector<Eigen::Vector2d> measurements);

    /**
     * @brief Deletes tracks that have a low probability of existence.
     * 
     * @param deletion_threshold The threshold for deleting a track.
     */
    void deleteTracks(double deletion_threshold);

    /**
     * @brief Sets the dynamic model for estimating target motion.
     * 
     * @param std_velocity The standard deviation of the target velocity.
     */
    void set_dyn_model(double std_velocity);

    /**
     * @brief Sets the sensor model for estimating target measurements.
     * 
     * @param std_measurement The standard deviation of the target measurement.
     */
    void set_sensor_model(double std_measurement);

    /**
     * @brief Retrieves the current tracks.
     * 
     * @return A vector of Track objects representing the current tracks.
     */
    std::vector<Track> getTracks() const { return tracks_; }

private:
    std::vector<Track> tracks_; ///< The vector of tracks.

    std::shared_ptr<DynMod> dyn_model_; ///< The dynamic model for estimating target motion.

    std::shared_ptr<SensorMod> sensor_model_; ///< The sensor model for estimating target measurements.

    int tracker_id_; ///< The tracker id.
};