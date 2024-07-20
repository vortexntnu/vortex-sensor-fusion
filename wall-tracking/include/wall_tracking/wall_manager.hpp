#pragma once

#include <Eigen/Dense>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_filtering/filters/ipda.hpp>
#include <wall_tracking/wall_model.hpp>

using State4d = vortex::prob::Gauss4d;
using DynMod = vortex::models::Wall;

using SensorMod = vortex::models::IdentitySensorModel<4, 4>;
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;

enum LandmarkAction {
    REMOVE_ACTION = 0,
    ADD_ACTION = 1,
    UPDATE_ACTION = 2
};

struct Wall {
    int id;
    State4d state;
    double existence_probability;
    bool confirmed;
    uint8_t action;

    // For sorting tracks based on existence probability and confirmed track
    bool operator<(const Wall &other) const {
        if (confirmed != other.confirmed) {
            return confirmed > other.confirmed;
        } else {
            return existence_probability < other.existence_probability;
        }
    }

    bool operator==(const Wall &other) const {
        return confirmed == other.confirmed && existence_probability == other.existence_probability;
    }
};


class WallManager {
public:

    /**
     * @brief Default constructor for the WallManager class.
     */
    WallManager();

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
    void updateWalls(Eigen::Array<double, 4, Eigen::Dynamic> measurements_, int update_interval, double confirmation_threshold, double gate_theshhold, double min_gate_threshold, double max_gate_threshold, double prob_of_detection, double prob_of_survival, double clutter_intensity, double initial_existence_probability);

    /**
     * @brief Creates new tracks for every measurements.
     * 
     * @param measurements The measurements received.
     */
    void createWalls(Eigen::Array<double, 4, Eigen::Dynamic> measurements, double initial_existence_probability);

    /**
     * @brief Deletes tracks that have a low probability of existence.
     * 
     * @param deletion_threshold The threshold for deleting a track.
     */
    void deleteWalls(double deletion_threshold);

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
     * @return A vector of Wall objects representing the current tracks.
     */
    std::vector<Wall> getWalls() const { return walls_; }

private:
    std::vector<Wall> walls_; ///< The vector of tracks.

    std::shared_ptr<DynMod> dyn_model_; ///< The dynamic model for estimating target motion.

    std::shared_ptr<SensorMod> sensor_model_; ///< The sensor model for estimating target measurements.

    int wall_id_inc_; ///< The tracker id.
};