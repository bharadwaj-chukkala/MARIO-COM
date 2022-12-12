/**
 * @file Navigation.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief Navigation class interface
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "./Perception.hpp"

#include<iostream>
#include <vector>

/**
 * @brief Navigation class to generate the search path of the robot and move it. The Perception class is a friend class of this.
 *      
 */
class Navigation {
 public:
    /**
     * @brief Construct a new Navigation object
     * 
     */
    Navigation();

    /**
     * @brief Member function to execute the search algorithm in the map.
     * 
     * @param map Map of the robot's environment.
     */
    void search_bins(std::vector<int> map);

    /**
     * @brief Member function to move the robot & bin to the disposal zone. 
     * 
     * @return true If the robot successfully reaches the disposal zone
     * @return false If the robot cannot reach the disposal zone
     */
    bool move_to_disposal_zone();

    /**
     * @brief Member function to resume search algorithm after disposal. 
     * 
     * @param prev_bin_pose Pose of the bin previously disposed.
     * @return true If the search can be resumed
     * @return false If the search cannot be resumed
     */
    bool resume_search(geometry_msgs::msg::Pose prev_bin_pose);

 private:
    geometry_msgs::msg::Pose m_curr_pose;
    geometry_msgs::msg::Pose m_next_pose;
    friend class Perception;
};
