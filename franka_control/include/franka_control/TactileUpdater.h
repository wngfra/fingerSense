// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <array>

#include <rclcpp/rclcpp.hpp>

#include "tactile_msgs/msg/tactile_signal.hpp"

using std::placeholders::_1;

class TactileUpdater : public rclcpp::Node
{
public:
    TactileUpdater(float *reduced_average) : Node("tactile_updater")
    {
        reduced_average_ = reduced_average;
        subscription_ = this->create_subscription<tactile_msgs::msg::TactileSignal>("/tactile_signals", 30, std::bind(&TactileUpdater::subscription_callback, this, _1));
    }

private:
    void subscription_callback(const tactile_msgs::msg::TactileSignal::SharedPtr msg) const;

    float *reduced_average_;
    rclcpp::Subscription<tactile_msgs::msg::TactileSignal>::SharedPtr subscription_;
};