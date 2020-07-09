// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <array>
#include <iostream>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "franka_control/TactileListener.h"
#include "tactile_msgs/msg/tactile_signal.hpp"

using std::placeholders::_1;

namespace franka_control
{
    void TactileListener::init(std::array<int, 16> *bufPtr)
    {
        bufPtr_ = bufPtr;

        subscription_ = this->create_subscription<tactile_msgs::msg::TactileSignal>(
            "/tactile_signals",
            10,
            std::bind(&TactileListener::topicCallback, this, _1));
    }

    void TactileListener::topicCallback(const tactile_msgs::msg::TactileSignal::SharedPtr msg) const
    {
        *bufPtr_ = msg->data;
    }
} // namespace franka_control