/**
 * @file RobotSim.cpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief RobotSim class implementation (code stub)
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/mario_com/RobotSim.hpp"

RobotSim::RobotSim() {
    m_nav = Navigation();
    m_perc = Perception();
    m_manip = Manipulation();
}
