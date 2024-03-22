#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

void writeIntsToFile(std::string filename, std::vector<int> ids);

std::string ros2TimeToString(const rclcpp::Time& time);
