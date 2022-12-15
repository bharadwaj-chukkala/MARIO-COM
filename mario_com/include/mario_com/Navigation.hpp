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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>
#include <iostream>
#include <vector>
#include <chrono>

using POSE = geometry_msgs::msg::PoseStamped;
using PUBLISHER = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;
using ODOM = nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::chrono::duration;
using namespace std::chrono_literals;
/**
 * @brief Navigation class to generate the search path of the robot and move it. The Perception class is a friend class of this.
 *      
 */
class Navigation : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Navigation object
     * 
     */
    Navigation();

    /**
     * @brief Member function to execute the search algorithm in the map.
     * 
     */
    bool search_bins();

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
     * @return true If the search can be resumed
     * @return false If the search cannot be resumed
     */
    bool resume_search();
    /**
     * @brief Call back for odom topic while searching
     * 
     * @param msg ODOM message
     */
    void odom_callback_search(const ODOM::SharedPtr msg);

    /**
     * @brief Call back for odom topic while disposing the bin
     * 
     * @param msg ODOM message
     */
    void odom_callback_disposal(const ODOM::SharedPtr msg);

    /**
     * @brief Call back for odom topic while resuming search
     * 
     * @param msg ODOM message
     */
    void odom_callback_resume(const ODOM::SharedPtr msg);

 private:
    PUBLISHER nav_publisher_;
    TIMER timer_;
    std::shared_ptr<rclcpp::Node> node_odom_nav;
    bool check_odom;
    float_t req_pos_y;
};
