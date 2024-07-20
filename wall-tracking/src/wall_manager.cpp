#include <wall_tracking/wall_manager.hpp>

WallManager::WallManager()
: wall_id_inc_(0)
{
}

void WallManager::updateWalls(Eigen::Array<double, 4, Eigen::Dynamic> measurements_,
    int update_interval, 
    double confirmation_threshold, 
    double gate_theshhold, 
    double min_gate_threshold, 
    double max_gate_threshold, 
    double prob_of_detection, 
    double prob_of_survival, 
    double clutter_intensity,
    double initial_existence_probability)
{
    // Sorts the tracks based on existence probability and confirmed wall
    std::sort(walls_.begin(), walls_.end());
    
    for (auto &wall : walls_)
    {
        IPDA::Config config;
        config.pdaf.mahalanobis_threshold = gate_theshhold;
        config.pdaf.min_gate_threshold = min_gate_threshold;
        config.pdaf.max_gate_threshold = max_gate_threshold;
        config.pdaf.prob_of_detection = prob_of_detection;
        config.pdaf.clutter_intensity = clutter_intensity;
        config.ipda.prob_of_survival = prob_of_survival;

        IPDA::State state_est_prev;
        state_est_prev.x_estimate = wall.state;
        state_est_prev.existence_probability = wall.existence_probability;
        // Predict next state
        auto output = 
            IPDA::step(*dyn_model_, 
            *sensor_model_,
            update_interval / 1000.0,
            state_est_prev, 
            measurements_, 
            config);
        // Update state
        wall.state = output.state.x_estimate;
        // Update existence probability
        wall.existence_probability = output.state.existence_probability;

        // Update wall existence
        if (wall.confirmed == false && output.state.existence_probability > confirmation_threshold)
        {
            wall.confirmed = true;
            wall.action = LandmarkAction::ADD_ACTION;
        }
        else if (wall.confirmed == true)
        {
            wall.action = LandmarkAction::UPDATE_ACTION;
        }


        // Update the measurement list
        Eigen::Array<double, 4, Eigen::Dynamic> outside(4, measurements_.cols());
        Eigen::Index inside_num = 0;
        for (Eigen::Index i = 0; i < measurements_.cols(); ++i)
        {
            if (output.gated_measurements[i])
            {
                inside_num++;
            }
            else
            {
                outside.col(i-inside_num) = measurements_.col(i);
            }
        }
        outside.conservativeResize(4, measurements_.cols() - inside_num);
        if(inside_num != 0)
        {
            measurements_ = outside;
        }
    }
    // Create new tracks based on the remaining measurements
    createWalls(measurements_, initial_existence_probability);
}

void WallManager::createWalls(Eigen::Array<double, 4, Eigen::Dynamic> measurements, 
    double initial_existence_probability)
{
        
    for (Eigen::Index i = 0; i < measurements.cols(); ++i)
    {
        Eigen::Vector4d state_estimate;
        state_estimate << measurements.col(i);
        Wall wall;
        wall.id = wall_id_inc_;
        wall.state = vortex::prob::Gauss4d(state_estimate, Eigen::Matrix4d::Identity());
        wall.existence_probability = initial_existence_probability;
        wall.confirmed = false;
        walls_.push_back(wall);
        wall_id_inc_++;
    }
}

void WallManager::deleteWalls(double deletion_threshold)
{
    walls_.erase(std::remove_if(walls_.begin(), walls_.end(), [deletion_threshold](const Wall &wall) { return wall.existence_probability < deletion_threshold; }), walls_.end());
}

void WallManager::set_dyn_model(double std_velocity)
{
    dyn_model_ = std::make_shared<DynMod>(std_velocity);
}

void WallManager::set_sensor_model(double std_measurement)
{
    sensor_model_ = std::make_shared<SensorMod>(std_measurement);
}