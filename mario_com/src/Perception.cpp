/**
 * @file Perception.cpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief Perception class implementation (code stub)
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/mario_com/Perception.hpp"
#include <functional>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include "./Navigation.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

Perception::Perception() : Node("perception") {
    // Initialize the publishers, subscribers for image processing.
    img_node = rclcpp::Node::make_shared("image_listener", options);
    m_pub_vel = this->create_publisher<TWIST>("cmd_vel", 10);
    percep_odom_node = rclcpp::Node::make_shared("percep_odom_node");
    odom_sub = percep_odom_node->create_subscription<ODOM>("odom", 10,
        std::bind(&Perception::odom_callback_search, this, _1));
}

void Perception::img_callback(const
            sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        // Convert the cv_bridge image to opencv Image.
        // cv::imshow("view", cv_bridge::toCvShare(msg, msg->encoding)->image);
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
        // HSV Masking to detect the trash bin.
        int low_H = 20, low_S = 100, low_V = 100;
        int high_H = 30, high_S = 255, high_V = 255;
        cv::Mat hsv, thr, bin;
        cv::cvtColor(image, hsv, CV_RGB2HSV);
        // Get the HSV Mask
        cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V),
            cv::Scalar(high_H, high_S, high_V), thr);
        // Apply thresholding to convert to binary
        threshold(thr, bin, 100, 255, cv::THRESH_BINARY);
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = thr.clone();
        // Find the contours in the image
        cv::findContours(contourOutput, contours, CV_RETR_LIST,
                    CV_CHAIN_APPROX_NONE);
        cv::Mat contourImage(image.size(), CV_8UC3,
                    cv::Scalar(0, 0, 0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);
        // Draw the contours around the detected bin
        if (contours.size() > 0) {
            for (size_t idx = 0; idx < contours.size(); idx++) {
                cv::drawContours(contourImage, contours,
                        idx, colors[idx % 3]);
            }
            // Create a bounding rectangle to get the center of the bin
            cv::Rect rect;
            rect = cv::boundingRect(contours.at(0));
            int cent_x = static_cast<int>((rect.x+rect.width)/2);
            int area = static_cast<int>(rect.area());
            // Decide if the robot needs to rotate left/right/stop/move_forward
            if (cent_x < 180) {
                l_rotate_flag = true;
                r_rotate_flag = false;
            } else if (cent_x > 200) {
                    r_rotate_flag = true;
                    l_rotate_flag = false;
            } else {
                if (area > 50000) {
                    stop_flag = true;
                    move_forward = false;
                } else {
                    move_forward = true;
                    stop_flag = false;
                }
                l_rotate_flag = false;
                r_rotate_flag = false;
            }
            // Check the yaw to decide if the robot is rotated to detect bins
        } else {
            if (present_yaw - initial_yaw > 1.57) {
                next_location = true;
                l_rotate_flag = false;
                r_rotate_flag = false;
                move_forward = false;
                stop_flag = true;
            } else {
                l_rotate_flag = true;
                r_rotate_flag = false;
                move_forward = false;
                stop_flag = false;
            }
        }
        // Uncomment the cv:: commented lines to visualize the output
        // cv::imshow("view", contourImage);
        // cv::waitKey(10);
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(),
            "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void Perception::odom_callback_search(const ODOM::SharedPtr msg) {
    // Convert the odom pose from quaternion to RPY angles
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    present_yaw = y;
}

bool Perception::detect_bin() {
    RCLCPP_INFO(this->get_logger(), "In Detect Bin");
    // Call the image call back to process the images from robot
    rclcpp::spin_some(percep_odom_node);
    initial_yaw = present_yaw;
    r_rotate_flag = false;
    l_rotate_flag = false;
    move_forward = false;
    stop_flag = false;
    next_location = false;

    // cv::namedWindow("view");
    // cv::startWindowThread();

    image_transport::ImageTransport it(img_node);
    sub = it.subscribe("pi_camera/image_raw", 1,
        std::bind(&Perception::img_callback, this, _1));
    // Start detecting the bin
    while (true) {
        rclcpp::spin_some(percep_odom_node);
        rclcpp::spin_some(img_node);
        if (stop_flag) {
            move_to_bin();
            break;
        } else if (next_location) {
            move_to_bin();
            return false;
        } else {
            move_to_bin();
            rclcpp::sleep_for(100ms);
        }
    }
    return true;
}


bool Perception::move_to_bin() {
    // Publish the velocities to the robot either to move forward or rotate.
    auto vel = TWIST();
    if (r_rotate_flag) {
        vel.angular.z = -0.1;
        vel.linear.x = 0;
    } else if (l_rotate_flag) {
        vel.angular.z = 0.1;
        vel.linear.x = 0;
    } else if (move_forward) {
        vel.linear.x = 0.1;
        vel.angular.z = 0;
    } else if (stop_flag) {
        vel.linear.x = 0;
        vel.angular.z = 0;
    }
    m_pub_vel->publish(vel);
    return true;
}
