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

Perception::Perception() : Node("perception") {
    node = rclcpp::Node::make_shared("image_listener", options);
    m_pub_vel = this->create_publisher<TWIST>("cmd_vel", 10);
}

void Perception::img_callback(const
            sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        RCLCPP_INFO(this->get_logger(), "In img call back");
        // cv::imshow("view", cv_bridge::toCvShare(msg, msg->encoding)->image);
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
        // RCLCPP_INFO(this->get_logger(), "%d", image.dims);
        int low_H = 20, low_S = 100, low_V = 100;
        int high_H = 30, high_S = 255, high_V = 255;
        cv::Mat hsv, thr, bin;
        cv::cvtColor(image, hsv, CV_RGB2HSV);

        cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V),
            cv::Scalar(high_H, high_S, high_V), thr);
        threshold(thr, bin, 100, 255, cv::THRESH_BINARY);
        // RCLCPP_INFO(this->get_logger(), "%d", contours.size());
        // cv::Moments mom;
        // cv::moments(bin, true);
        // cv::Point p(mom.m10/mom.m00, mom.m01/mom.m00);

        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = thr.clone();
        cv::findContours(contourOutput, contours, CV_RETR_LIST,
                    CV_CHAIN_APPROX_NONE);
        cv::Mat contourImage(image.size(), CV_8UC3,
                    cv::Scalar(0, 0, 0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);
        if (contours.size() > 0) {
            for (size_t idx = 0; idx < contours.size(); idx++) {
                cv::drawContours(contourImage, contours,
                        idx, colors[idx % 3]);
            }
            cv::Rect rect;
            rect = cv::boundingRect(contours.at(0));
            int cent_x = static_cast<int>((rect.x+rect.width)/2);
            int area = static_cast<int>(rect.area());
            RCLCPP_INFO(this->get_logger(), "Area %d", area);
            if (cent_x < 180) {
                l_rotate_flag = true;
                r_rotate_flag = false;
            } else if (cent_x > 300) {
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
        } else {
            l_rotate_flag = true;
            r_rotate_flag = false;
            move_forward = false;
            stop_flag = false;
        }
        // RCLCPP_INFO(this->get_logger(), "%d %d %d %d",
        //     static_cast<int>((rect.x+rect.width)/2),
        //         static_cast<int>((rect.y+rect.height)/2),
        //         contourImage.size().width, contourImage.size().height);
        cv::imshow("view", contourImage);
        cv::waitKey(10);
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(),
            "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

bool Perception::detect_bin() {
    r_rotate_flag = false;
    l_rotate_flag = false;
    move_forward = false;
    stop_flag = false;

    cv::namedWindow("view");
    cv::startWindowThread();

    image_transport::ImageTransport it(node);
    sub = it.subscribe("pi_camera/image_raw", 1,
        std::bind(&Perception::img_callback, this, _1));

    while (true) {
        rclcpp::spin_some(node);
        if (stop_flag) {
            break;
        } else {
            move_to_bin();
        }
    }

    return true;
}


bool Perception::move_to_bin() {
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

    // vel.linear.y = 0.0;
    // vel.linear.z = 0.0;
    // vel.angular.x = 0.0;
    // vel.angular.y = 0.0;
    m_pub_vel->publish(vel);
    return true;
}
