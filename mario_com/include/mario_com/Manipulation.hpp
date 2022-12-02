/**
 * @file Manipulation.hpp
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

class Manipulation {
 public:
    Manipulation();
    void gripper_open();
    void gripper_close();
    bool pick_bin();
    bool place_bin();
 private:
    geometry_msgs::msg::Pose m_pick_pose;
    geometry_msgs::msg::Pose m_place_pose;
}
