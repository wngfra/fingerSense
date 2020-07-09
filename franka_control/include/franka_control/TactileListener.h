// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "tactile_msgs/msg/tactile_signal.hpp"

namespace franka_control
{
    class TactileListener : public rclcpp::Node
    {
    public:
        TactileListener(std::array<int, 16> *bufPtr) : Node("tactile_listener")
        {
            init(bufPtr);
        }

    private:
        void init(std::array<int, 16> *bufPtr);
        void topicCallback(const tactile_msgs::msg::TactileSignal::SharedPtr msg) const;

        rclcpp::Subscription<tactile_msgs::msg::TactileSignal>::SharedPtr subscription_;

        std::array<int, 16> *bufPtr_ = nullptr;
    };
} // namespace franka_control
