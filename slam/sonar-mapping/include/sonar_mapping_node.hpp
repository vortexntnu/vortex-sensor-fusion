
#ifndef SONAR_MAPPING_NODE  
#define SONAR_MAPPING_NODE

// Dependencies
#include <ros/ros.h>
#include "math_functions.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class sonarMapping {

private:
    // ROS handle
    ros::NodeHandle nh;

    // Variables to store messages from callback functions
    sensor_msgs::LaserScan sonar_data;
    nav_msgs::OccupancyGrid map_data;
    nav_msgs::Odometry ekf_data;

    // Publisher
    ros::Publisher map_pub;

    // Subscribers
    ros::Subscriber ekf_sub;
    ros::Subscriber sonar_sub;

    // Callback timer
    ros::Timer timer;

    // Map
    MatrixXi global_map;  

    // ROSparams
    float rate;
    int scale, size;
    string laser_topic, odometry_topic, map_topic;

public:
    // Constructor
    sonarMapping(int argc, char** argv) {
        // Get ROSparams
        nh.getParam("PUBLISHING_RATE",rate);
        nh.getParam("MAP_SIZE",size);
        nh.getParam("SCALE",scale);
        nh.getParam("LASER_SCAN_TOPIC", laser_topic);
        nh.getParam("ODOMETRY_TOPIC", odometry_topic);
        nh.getParam("MAP_PUBLISHING_TOPIC", map_topic);
        // Initiates map
        global_map = MatrixXi(size,size);
        global_map.setConstant(-1);
        // Initate msg
        initiateMapMsg();
        //Initiate publsiher
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic,100);
    
        // Callback functions
        ekf_sub = nh.subscribe(odometry_topic,100,&sonarMapping::ekfCallback, this);
        sonar_sub = nh.subscribe(laser_topic,100,&sonarMapping::sonarCallback, this);
        timer = nh.createTimer(ros::Duration(rate),&sonarMapping::mappingCallback, this);
    }
    // Destructor
    ~sonarMapping(){

    }

    // Callback functions
    void sonarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void ekfCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mappingCallback(const ros::TimerEvent& event);

    // Mapping function
    void updateGlobalMap(float scale);

    // Globalmap message functions
    void initiateMapMsg();
    void updateMapMsg();
    

};

#endif