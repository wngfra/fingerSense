// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "tactile_interfaces/srv/change_state.hpp"

namespace franka_control
{
    class NodeStateManager
    {
    public:
        NodeStateManager(const std::string&, const std::string&);
        // ~NodeStateManager();

        void change_state(const int, const std::chrono::nanoseconds);

    private:
        // Node-state manager node handler
        std::shared_ptr<rclcpp::Node> nh;
        rclcpp::Client<tactile_interfaces::srv::ChangeState>::SharedPtr client;
        std::shared_ptr<tactile_interfaces::srv::ChangeState::Request> request;
    };
} // namespace franka_control
