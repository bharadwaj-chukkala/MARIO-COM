/**
 * @file Navigation.hpp
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
#include "geometry_msgs/msg/pose.hpp"
#include "./Perception.hpp"

class Navigation {
 public:
    Navigation();
    void search_bins(auto map);
    bool move_to_disposal_zone();
    bool resume_search(geometry_msgs::msg::Pose prev_bin_pose);
 private:
    geometry_msgs::msg::Pose m_curr_pose;
    geometry_msgs::msg::Pose m_next_pose;
    friend class Perception;
}
