// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <thread>

#include "franka_control/NodeStateManager.h"

namespace franka_control
{
    NodeStateManager::NodeStateManager(const std::string &node_name, const std::string &service_name)
    {
        if (rclcpp::ok())
        {
            nh = rclcpp::Node::make_shared(node_name);
            client = nh->create_client<tactile_interfaces::srv::ChangeState>(service_name);
            request = std::make_shared<tactile_interfaces::srv::ChangeState::Request>();
        }
    }

    bool NodeStateManager::change_state(const int new_state, const std::chrono::nanoseconds time_out)
    {
        request->transition = new_state;

        // timeout in nanoseconds
        rclcpp::sleep_for(time_out);

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(nh, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }
} // namespace franka_control