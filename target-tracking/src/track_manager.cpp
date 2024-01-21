#include <target_tracking/track_manager.hpp>

TrackManager::TrackManager(double clutter_rate, double probability_of_detection, double gate_threshold, double std_velocity, double std_sensor)
: dyn_model_(std::make_shared<DynMod>(std_velocity))
, sensor_model_(std::make_shared<SensorMod>(std_sensor))
, pdaf_(std::make_shared<PDAF>(gate_threshold, probability_of_detection, clutter_rate))
, tracker_id_(0)
{
    
}

void TrackManager::updateTracks(std::vector<Eigen::Vector2d> measurements, int update_interval)
{
    // Sorts the tracks based on existence probability and confirmed track
    std::sort(tracks_.begin(), tracks_.end());

    std::vector<Eigen::Vector2d> z_meas;

    for (auto &track : tracks_)
    {
        // Predict next state (PDAF)
        auto [x_pred, inside, outside] = pdaf_->predict_next_state(track.state, z_meas, update_interval / 1000.0, dyn_model_, sensor_model_);

        // Update state
        track.state = x_pred;

        // Update the measurement list
        measurements = outside;

        // Update existence probability (IPDA)
        
        
    }

    double deletion_threshold = 0.2;
    double confirmation_threshold = 0.8;

    // Update confirmed tracks and delete unwanted tracks
    for (auto &track : tracks_)
    {
        if (track.confirmed == false && track.existence_probability > confirmation_threshold)
        {
            track.confirmed = true;
        }
        else if (track.existence_probability < deletion_threshold)
        {
            tracks_.erase(std::remove(tracks_.begin(), tracks_.end(), track), tracks_.end());
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