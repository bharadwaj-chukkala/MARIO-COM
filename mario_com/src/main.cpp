/**
 * @file main.cpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief Main file for simulation and demo.
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/mario_com/RobotSim.hpp"
// # include "rclcpp/rclcpp.hpp"
int main(int argc, char * argv[]) {
    // Code stub
    rclcpp::init(argc, argv);
    RobotSim rs;
    // rclcpp::spin(rs.m_nav);
    rs.m_nav.move_to_disposal_zone();
    rs.m_nav.resume_search();
    rclcpp::shutdown();

    return 0;
}
