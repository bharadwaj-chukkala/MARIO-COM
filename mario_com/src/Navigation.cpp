/**
 * @file Navigation.cpp
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
#include "../include/mario_com/Navigation.hpp"

Navigation::Navigation() {
    m_curr_pose = nullptr;
    m_next_pose = nullptr;
}

void Navigation::search_bins(auto map) {
    // Code Stub
}

bool Navigation::move_to_disposal_zone() {
    // Code Stub
    return true;
}

bool Navigation::resume_search(geometry_msgs::msg::Pose prev_bin_pose) {
    // Code Stub
    return true;
}
