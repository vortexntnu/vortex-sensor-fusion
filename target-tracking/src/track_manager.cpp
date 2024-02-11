#include <target_tracking/track_manager.hpp>

TrackManager::TrackManager()
: tracker_id_(0)
{
}

void TrackManager::updateTracks(std::vector<Eigen::Vector2d> measurements, int update_interval, double confirmation_threshold, double gate_theshhold, double prob_of_detection, double prob_of_survival, double clutter_intensity)
{
    // Sorts the tracks based on existence probability and confirmed track
    std::sort(tracks_.begin(), tracks_.end());
    std::cout << "tracks size: " << tracks_.size() << std::endl;
    for (auto &track : tracks_)
    {
        // Predict next state
        auto [x_final, existence_probability, inside, outside, x_pred, z_pred, x_updated] = IPDA::step(track.state, measurements, update_interval / 1000.0, dyn_model_, sensor_model_, gate_theshhold, prob_of_detection, prob_of_survival, track.existence_probability, clutter_intensity);

        // Update state
        track.state = x_final;

        // Update existence probability
        track.existence_probability = existence_probability;

        std::cout << "existence probabilities" << existence_probability << std::endl;

        // Update the measurement list
        measurements = outside;
    }

    // Update track existence
    for (auto &track : tracks_)
    {
        if (track.confirmed == false && track.existence_probability > confirmation_threshold)
        {
            track.confirmed = true;
        }
    }

    // Create new tracks based on the remaining measurements
    createTracks(measurements);

}

void TrackManager::createTracks(std::vector<Eigen::Vector2d> measurements)
{
    for (const auto &point : measurements)
    {
        Eigen::Vector4d state_estimate;
        state_estimate << point, 0.0, 0.0;
        Track track;
        track.id = tracker_id_;
        track.state = vortex::prob::Gauss4d(state_estimate, Eigen::Matrix4d::Identity());
        track.existence_probability = 0.4;
        track.confirmed = false;

        tracks_.push_back(track);

        tracker_id_++;

        // remove the point from the list of measurements
        measurements.erase(std::remove(measurements.begin(), measurements.end(), point), measurements.end());
    }
}

void TrackManager::deleteTracks(double deletion_threshold)
{
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [deletion_threshold](const Track &track) { return track.existence_probability < deletion_threshold; }), tracks_.end());
}

void TrackManager::set_dyn_model(double std_velocity)
{
    dyn_model_ = std::make_shared<DynMod>(std_velocity);
}

void TrackManager::set_sensor_model(double std_measurement)
{
    sensor_model_ = std::make_shared<SensorMod>(std_measurement);
}