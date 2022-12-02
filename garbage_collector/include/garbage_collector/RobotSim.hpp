/**
 * @file RobotSim.hpp
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
#include "./Manipulation.hpp"
#include "./Navigation.hpp"
#include "./Perception.hpp"

class RobotSim {
 public:
    RobotSim();
    Navigation m_nav;
    Perception m_perc;
    Manipulation m_manip;
}
