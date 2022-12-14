/**
 * @file Manipulation.cpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu) [Driver]
 * @author Adarsh Malapaka (amalapak@umd.edu) [Navigator]
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu) [Design Keeper]
 * @brief Manipulation class implementation (code stubs)
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/mario_com/Manipulation.hpp"
#include <fstream>
#include <ios>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>

Manipulation::Manipulation() : Node("manipulation") {
    m_place_pose.position.x = 3.5;
    m_place_pose.position.y = -2.5;
    m_place_pose.position.z = 0;
    m_place_pose.orientation.x = 0;
    m_place_pose.orientation.y = 0;
    m_place_pose.orientation.z = 0;
    m_place_pose.orientation.w = 0;
    pick_client = create_client<SERVICE_DELETE>("delete_entity");
    place_client = create_client<SERVICE_SPAWN>("spawn_entity");
    manip_node = rclcpp::Node::
                    make_shared("bin_node");
}

void Manipulation::gripper_open() {
    // Code Stub
}

void Manipulation::gripper_close() {
    // Code Stub
}



bool Manipulation::pick_bin() {
    while (!pick_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          // Used one of the RCLCPP LOG Level
          RCLCPP_ERROR(this->get_logger(),
              "Interruped while waiting for the server.");

          return false;
        }
        RCLCPP_INFO(this->get_logger(),
              "Server not available, waiting again...");
    }
    auto request = std::make_shared<REQUEST_DELETE>();
    request->name = "trash_bin1";

    // auto call_back_ptr = std::bind(&Manipulation::response_callback,
    //                       this, _1);
    auto result = pick_client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(manip_node,
                                            result, 10s);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    } else {
        return false;
    }
}

bool Manipulation::place_bin() {
    auto res = system("ros2 run gazebo_ros spawn_entity.py -entity trash_bin -x 3.5 -y -2.5 -z 0 -file `ros2 pkg prefix mario_com`/share/mario_com/models/bin_cylinder/model.sdf");
    if (res > 0) {
        return true;
    } else {
        return false;
    }
}
