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

using REQUEST = gazebo_msgs::srv::DeleteEntity::Request;
using SERVICE = gazebo_msgs::srv::DeleteEntity;
using CLIENT = rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr;
using RESPONSE = rclcpp::Client<SERVICE>::SharedFuture;
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
     * @brief Member function to open the gripper
     * 
     */
    void gripper_open();

    /**
     * @brief Member function to close the gripper
     * 
     */
    void gripper_close();

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

    void response_callback(RESPONSE future);

 private:
    geometry_msgs::msg::Pose m_pick_pose;
    geometry_msgs::msg::Pose m_place_pose;
    CLIENT client;
    rclcpp::Node::SharedPtr manip_node;
};
