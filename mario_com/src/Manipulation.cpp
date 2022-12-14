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
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>

Manipulation::Manipulation() : Node("manipulation") {
    m_pick_pose.position.x = 0.0;
    m_place_pose.position.x = 0.0;
    client = create_client<SERVICE>("delete_entity");
    manip_node = rclcpp::Node::
                    make_shared("bin_node");
}

void Manipulation::gripper_open() {
    // Code Stub
}

void Manipulation::gripper_close() {
    // Code Stub
}

void Manipulation::response_callback(RESPONSE future) {

    // // Get the data from the response
    // my_data = future.get()->output.c_str();
}


bool Manipulation::pick_bin() {
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          // Used one of the RCLCPP LOG Level
          RCLCPP_ERROR(this->get_logger(),
              "Interruped while waiting for the server.");

          return false;
        }
        RCLCPP_INFO(this->get_logger(),
              "Server not available, waiting again...");
    }
    auto request = std::make_shared<REQUEST>();
    request->name = "trash_bin1";

    // auto call_back_ptr = std::bind(&Manipulation::response_callback,
    //                       this, _1);
    auto result = client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(manip_node,
                                            result, 10s);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    } else {
        return false;
    }
}
bool Manipulation::place_bin() {
    // Code Stub
    return true;
}
