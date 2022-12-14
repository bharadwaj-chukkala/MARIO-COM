/**
 * @file Navigation.cpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief Navigation class implementation (code stubs) 
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/mario_com/Navigation.hpp"
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

bool check_odom = false;
float_t req_pos_y = 0.0;


void odom_callback_search(const ODOM::SharedPtr msg) {
    // RCLCPP_INFO(rclcpp::get_logger("log"), "In Search Bins odom");
    // RCLCPP_INFO(rclcpp::get_logger("log"), "Present %f Required %f",
                        // msg->pose.pose.position.y, req_pos_y);
    if ((std::abs(static_cast<int>(msg->pose.pose.position.y - req_pos_y))
        == 0) && (std::abs(static_cast<int>(msg->pose.pose.position.x)) == 0)) {
            check_odom = true;
            // RCLCPP_INFO(rclcpp::get_logger("log"), "In Search Bins odom");
    }
}

void odom_callback_disposal(const ODOM::SharedPtr msg) {
    if ((std::abs(static_cast<int>(msg->pose.pose.position.x - 3.5)) == 0)
        && (std::abs(static_cast<int>(msg->pose.pose.position.y + 2.5)) == 0)) {
            check_odom = true;
    }
}

void odom_callback_resume(const ODOM::SharedPtr msg) {
    if ((std::abs(static_cast<int>(msg->pose.pose.position.x)) == 0)
        && (std::abs(static_cast<int>(msg->pose.pose.position.y)) == 0)) {
            check_odom = true;
    }
}

Navigation::Navigation() : Node("navigation") {
    m_curr_pose.position.x = 0.0;
    m_next_pose.position.x = 0.0;
    node_odom_nav = rclcpp::Node::
                    make_shared("odom_node");
    nav_publisher_ = this->create_publisher<POSE>("goal_pose", 10);
}

bool Navigation::search_bins() {
    rclcpp::sleep_for(1000ms);
    std::vector<float_t> search_pos = {6.0, 4.0, 2.0, 0.0};
    // std::shared_ptr<rclcpp::Node> node_odom_nav = rclcpp::Node::
    //                 make_shared("odom_node");
    auto odom_sub = node_odom_nav->create_subscription<ODOM>("odom", 10,
                                            odom_callback_search);
    while (search_pos.size() > 0) {
        float_t pop_pos = search_pos.back();
        search_pos.pop_back();
        RCLCPP_INFO(this->get_logger(), "In Search Bins %d %f",
                    search_pos.size(), pop_pos);
        check_odom = false;
        req_pos_y = pop_pos;
        POSE rpyGoal;
        rpyGoal.header.frame_id = "map";
        rpyGoal.header.stamp = this->get_clock()->now();

        rpyGoal.pose.position.x = 0;
        rpyGoal.pose.position.y = pop_pos;
        rpyGoal.pose.position.z = 0;
        rpyGoal.pose.orientation.x = 0;
        rpyGoal.pose.orientation.y = 0;
        rpyGoal.pose.orientation.z = 0;
        rpyGoal.pose.orientation.w = 1;
        // while (true) {
        //     rclcpp::spin_some(node_odom_nav);
        // }
        while (!check_odom) {
            // RCLCPP_INFO(this->get_logger(), "In Search Bins while");
            // return false;
            // rclcpp::sleep_for(1000ms);
            rclcpp::spin_some(node_odom_nav);

            // RCLCPP_INFO(this->get_logger(), "HELLO");
            // rclcpp::sleep_for(1000ms);
            nav_publisher_->publish(rpyGoal);
            // RCLCPP_INFO(this->get_logger(), "After Publish Check");
            rclcpp::sleep_for(500ms);
            // RCLCPP_INFO(this->get_logger(), "Check");
        }
        return false;
    }
    return true;
}

bool Navigation::move_to_disposal_zone() {
    RCLCPP_INFO(this->get_logger(), "In Move to Disposal Zone");
    check_odom = false;
    auto odom_sub = node_odom_nav->create_subscription<ODOM>("odom", 10,
                                            odom_callback_disposal);
    POSE rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = this->get_clock()->now();
    // rpyGoal.header.stamp.nanosec = 0;
    rpyGoal.pose.position.x = 3.5;
    rpyGoal.pose.position.y = -2.5;
    rpyGoal.pose.position.z = 0;
    // tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = 0;
    rpyGoal.pose.orientation.w = 1;

    while (!check_odom) {
        rclcpp::spin_some(node_odom_nav);
        nav_publisher_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
        // RCLCPP_INFO(this->get_logger(), "Check");
    }
    return true;
}

bool Navigation::resume_search() {
    RCLCPP_INFO(this->get_logger(), "In move to resume search");
    check_odom = false;
    auto odom_sub = node_odom_nav->create_subscription<ODOM>("odom", 10,
                                            odom_callback_resume);
    POSE rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = this->get_clock()->now();
    // rpyGoal.header.stamp.nanosec = 0;
    rpyGoal.pose.position.x = 0;
    rpyGoal.pose.position.y = 0;
    rpyGoal.pose.position.z = 0;
    // tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = 0;
    rpyGoal.pose.orientation.w = 1;

    while (!check_odom) {
        rclcpp::spin_some(node_odom_nav);
        nav_publisher_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
        // RCLCPP_INFO(this->get_logger(), "Check");
    }
    rclcpp::shutdown();
    return true;
}
