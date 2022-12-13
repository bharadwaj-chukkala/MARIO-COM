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

bool check_odom = false;

Navigation::Navigation() : Node("navigation") {
    m_curr_pose.position.x = 0.0;
    m_next_pose.position.x = 0.0;
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

void Navigation::search_bins(std::vector<int> map) {
    // Code Stub
}

bool Navigation::move_to_disposal_zone() {
    check_odom = false;
    nav_publisher_ = this->create_publisher<POSE>("goal_pose", 10);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("odom_node");
    auto odom_sub = node->create_subscription<ODOM>("odom", 10,
                                            odom_callback_disposal);
    POSE rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp.sec = 0;
    rpyGoal.header.stamp.nanosec = 0;
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
        rclcpp::spin_some(node);
        nav_publisher_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
        // RCLCPP_INFO(this->get_logger(), "Check");
    }
    return true;
}

bool Navigation::resume_search() {
    check_odom = false;
    nav_publisher_ = this->create_publisher<POSE>("goal_pose", 10);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("odom_node");
    auto odom_sub = node->create_subscription<ODOM>("odom", 10,
                                            odom_callback_resume);
    POSE rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp.sec = 0;
    rpyGoal.header.stamp.nanosec = 0;
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
        rclcpp::spin_some(node);
        nav_publisher_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
        // RCLCPP_INFO(this->get_logger(), "Check");
    }
    return true;
}
