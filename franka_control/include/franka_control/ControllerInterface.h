// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "tactile_msgs/srv/change_state.hpp"

namespace franka_control
{
    class ControllerInterface
    {
    public:
        ControllerInterface();
        ~ControllerInterface();

    private:
        // Client to change tactile_publisher node state
        std::shared_ptr<rclcpp::Node> client_node;
        rclcpp::Client<tactile_msgs::srv::ChangeState>::SharedPtr client;
        
    };
} // namespace franka_control
