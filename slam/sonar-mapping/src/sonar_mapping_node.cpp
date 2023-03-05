#include "sonar_mapping_node.hpp"


// Callback reading Odometry message into ekf_data class variable
void sonarMapping::ekfCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ekf_data.pose.pose.position.x = msg->pose.pose.position.x;
    ekf_data.pose.pose.position.y = msg->pose.pose.position.y;
    ekf_data.pose.pose.position.z = msg->pose.pose.position.z;
    ekf_data.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    ekf_data.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    ekf_data.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    ekf_data.pose.pose.orientation.w = msg->pose.pose.orientation.w;
}

// Callback reading LaserScan message into sonar_data class variable
void sonarMapping::sonarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    sonar_data.angle_min = msg->angle_min;
    sonar_data.angle_max = msg->angle_max;
    sonar_data.angle_increment = msg->angle_increment;
    sonar_data.ranges = msg->ranges;
}

// Timer callback publishing a map at a set intervall 
void sonarMapping::mappingCallback(const ros::TimerEvent& event) {
    updateGlobalMap(scale);
    updateMapMsg();
    map_pub.publish(map_data);
    

}

void sonarMapping::updateGlobalMap(float scale) {
    // Declaring necesarry variables
    float angle_increment, current_angle, tmp_width, tmp_height;
    vector<float> ranges;
    Vector2i manta_position,rotated_point;
    Vector3f manta_angles;
    Vector2f point, center; 
    vector<int> x,y;
    line_list::iterator it;
    // Calculating current positions and euler angles of manta
    manta_position[0] = (int)(((float)global_map.rows())/2.0 - ekf_data.pose.pose.position.x*scale);
    manta_position[1] = (int)(((float)global_map.cols())/2.0 - ekf_data.pose.pose.position.y*scale);
    manta_angles = quaternion_to_euler(ekf_data.pose.pose.orientation.x,ekf_data.pose.pose.orientation.y,
        ekf_data.pose.pose.orientation.z,ekf_data.pose.pose.orientation.w);
    
    // Extracting sonar_data
    angle_increment = sonar_data.angle_increment;
    current_angle = sonar_data.angle_min;
    ranges = sonar_data.ranges;

    // Looping through all ranges drawing them on map
    for (int i = 0; i < ranges.size(); i++) {
        // To much pitch, or to long range will lead to reflections from the floor wish is bad for the algorithm
        if (ranges[i] < 15 && manta_angles[1] < 0.1) {
            // Projecting points in sonar frame
            tmp_width = sin(current_angle)*ranges[i];
            tmp_height = sqrt(pow(ranges[i],2.0) - pow(tmp_width,2.0));
            tmp_width = manta_position[1] - (tmp_width*scale);
            tmp_height = manta_position[0] - (tmp_height*scale);

            // Transforming point from sonar frame to auv frame
            point[0] = tmp_height;
            point[1] = tmp_width;
            center[0] = (float)manta_position[0];
            center[1] = (float)manta_position[1];

            rotated_point = rotatePointAroundCenter(point, center, manta_angles[2]);
          

            // Find all points between Manta and the current ping and marks them as empty if no obstacle is detected
            lineBetweenPoints2(rotated_point[0],rotated_point[1],manta_position[0],manta_position[1],x,y);

            for (int i = 0; i < x.size(); i++) {
                if (global_map(y[i],x[i]) != 100 && global_map(y[i],x[i]) != 0) {
                    global_map(y[i],x[i]) = 0;
                }

            }
            /*
            for (it=line_points.begin();it!=line_points.end();it++) {
            
                if (global_map(get<0>(*it),get<1>(*it)) != 100 && global_map(get<0>(*it),get<1>(*it)) != 0) {
                    global_map(get<0>(*it),get<1>(*it)) = 0;

                }
            }
            */
            global_map(rotated_point[0],rotated_point[1]) = 100;
        }
        current_angle += angle_increment;
    }
    
}


void sonarMapping::initiateMapMsg() {
    map_data.header.frame_id = "map";
    map_data.info.resolution = (1.0/(float)scale);
    map_data.info.width = size;
    map_data.info.height = size;
    map_data.info.origin.position.x = - ((float)size/2.0) * (1.0/(float)scale);
    map_data.info.origin.position.y = - ((float)size/2.0) * (1.0/(float)scale);
    map_data.header.stamp = ros::Time::now();
    map_data.header.frame_id = "manta/odom";  
    map_data.data.resize(size*size);
}


void sonarMapping::updateMapMsg() {
    // updating data field in occupancy grid msg
    MatrixXi tmp_v;
    //tmp_v = global_map.transpose();
    tmp_v = global_map.reverse();
    tmp_v.resize(1,size*size);
    for (int i = 0; i < tmp_v.size();i++) {
        map_data.data[i] = tmp_v(i);
    }
    map_data.header.stamp = ros::Time::now();
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_mapping");
    sonarMapping ic(argc, argv);
    ros::spin();
    return 0;
}