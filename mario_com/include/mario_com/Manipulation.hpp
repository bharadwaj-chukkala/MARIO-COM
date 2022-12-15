/**
 * @file Manipulation.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief Manipulation class interface
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include <gazebo_msgs/srv/detail/delete_entity__struct.hpp>
#include <fstream>
#include <iostream>

using REQUEST_DELETE = gazebo_msgs::srv::DeleteEntity::Request;
using SERVICE_DELETE = gazebo_msgs::srv::DeleteEntity;
using CLIENT_DELETE = rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr;
using RESPONSE_DELETE = rclcpp::Client<SERVICE_DELETE>::SharedFuture;

using REQUEST_SPAWN = gazebo_msgs::srv::SpawnEntity::Request;
using SERVICE_SPAWN = gazebo_msgs::srv::SpawnEntity;
using CLIENT_SPAWN = rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr;
using RESPONSE_SPAWN = rclcpp::Client<SERVICE_DELETE>::SharedFuture;

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief Manipulation class to command the gripper to pick and place the disposal bin.
 * 
 */
class Manipulation : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Manipulation object
     * 
     */
    Manipulation();

    /**
     * @brief Member function to pick the bin.
     * 
     * @return true If bin is successfully picked
     * @return false If bin cannot be picked
     */
    bool pick_bin();

    /**
     * @brief Member function to place the bin.
     * 
     * @return true If bin is successfully placed
     * @return false If bin cannot be placed
     */
    bool place_bin();


 private:
    geometry_msgs::msg::Pose m_place_pose;
    CLIENT_DELETE pick_client;
    CLIENT_SPAWN place_client;
    rclcpp::Node::SharedPtr manip_node;
};
