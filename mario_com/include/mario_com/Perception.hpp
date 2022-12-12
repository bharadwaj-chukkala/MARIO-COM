/**
 * @file Perception.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief Perception class interface
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @brief Perception class to detect the bins using image processing.
 * 
 */
class Perception {
 public:
    /**
     * @brief Construct a new Perception object
     * 
     */
    Perception();

    /**
     * @brief Member function to detect the disposal bin.
     * 
     * @return true If the bin is detected.
     * @return false If the bin is not detected.
     */
    bool detect_bin();

    /**
     * @brief Member function to select a bin and orient the bot to make the bin appear in the center of the image frame.
     * 
     * @param pose Pose of the robot.
     */
    void select_bin(geometry_msgs::msg::Pose pose);

    /**
     * @brief Member function to move the robot towards the bin using camera and LiDAR data.
     * 
     * @return true If robot reaches the bin.
     * @return false If robot doesn't reach the bin.
     */
    bool move_to_bin();

 private:
    cv::Mat m_img_feed;
    sensor_msgs::msg::LaserScan m_lidar_feed;
};
