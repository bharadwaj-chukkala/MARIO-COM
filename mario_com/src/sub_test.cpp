#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>
#include<iostream>
#include <vector>

using POSE = geometry_msgs::msg::PoseStamped;
using PUBLISHER = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;
using ODOM = nav_msgs::msg::Odometry;
using std::placeholders::_1;


void topic_callback(const ODOM::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Logger"), "I heard");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto g_node = rclcpp::Node::make_shared("minimal_subscriber");
  auto subscription =
    g_node->create_subscription<ODOM>("odom", 10, topic_callback);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}