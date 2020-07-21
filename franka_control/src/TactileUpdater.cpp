// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <array>

#include <rclcpp/rclcpp.hpp>

#include "tactile_msgs/msg/tactile_signal.hpp"

#include "franka_control/TactileUpdater.h"

void TactileUpdater::subscription_callback(const tactile_msgs::msg::TactileSignal::SharedPtr msg) const
{
    // Computes the average tactile response
    auto buffer = msg->data;
    float reduced_average = std::accumulate(buffer.begin(), buffer.end(), 0.0, [&](float reduced_average, int e) {
        return reduced_average + (float)e / (float)buffer.size();
    });

    *reduced_average_ = reduced_average;
}