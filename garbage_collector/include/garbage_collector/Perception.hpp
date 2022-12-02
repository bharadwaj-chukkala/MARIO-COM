/**
 * @file Perception.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief 
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <rclcpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Perception {
 public:
    Perception();
    bool detect_bin();
    void select_bin(geometry_msgs::msg::Pose pose);
    bool move_to_bin();
 private:
    cv::Mat m_img_feed;
    sensor_msgs::LaserScan m_lidar_feed;
}
