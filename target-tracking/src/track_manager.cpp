#include <target_tracking/track_manager.hpp>

TrackManager::TrackManager()
: pdaf_()
, tracker_id_(0)
{
}

void TrackManager::updateTracks(std::vector<Eigen::Vector2d> measurements, int update_interval, double confirmation_threshold, double gate_theshhold, double prob_of_detection, double clutter_intensity)
{
    // Sorts the tracks based on existence probability and confirmed track
    std::sort(tracks_.begin(), tracks_.end());

    for (auto &track : tracks_)
    {
        // Predict next state (PDAF)
        auto [x_final, inside, outside, x_pred, z_pred, x_updated] = pdaf_->predict_next_state(track.state, measurements, update_interval / 1000.0, dyn_model_, sensor_model_, gate_theshhold, prob_of_detection, clutter_intensity);

        // Update state
        track.state = x_final;

        // Update the measurement list
        measurements = outside;

        // Update existence probability (IPDA)
        if (inside.size() > 0)
        {
            track.existence_probability += 0.1; 
        } else {
            track.existence_probability -= 0.1;
        }
        // track.existence_probability = ipda_ -> update_existence_probability(track.existence_probability, inside.size(), outside.size(), prob_of_detection, clutter_intensity);
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
    for (const auto &point : measurements)
        {
            Eigen::Vector4d state_estimate;
            state_estimate << point, 0.0, 0.0;
            Track track;
            track.id = tracker_id_;
            track.state = vortex::prob::Gauss4d(state_estimate, Eigen::Matrix4d::Identity());
            track.existence_probability = 0.5;
            track.confirmed = false;

            tracks_.push_back(track);

            tracker_id_++;

            // remove the point from the list of measurements
            measurements.erase(std::remove(measurements.begin(), measurements.end(), point), measurements.end());
        }
}

void TrackManager::deleteTracks(double deletion_threshold)
{
    // Delete tracks with existence probability below the deletion threshold
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