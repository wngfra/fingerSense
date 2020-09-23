// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <array>

#include <rclcpp/rclcpp.hpp>

#include "franka_msgs/msg/sliding_control.hpp"
#include "tactile_msgs/msg/tactile_signal.hpp"

using std::placeholders::_1;

class TactileUpdater : public rclcpp::Node
{
public:
    TactileUpdater(float *reduced_average, float *sliding_control) : Node("tactile_updater")
    {
        reduced_average_ = reduced_average;
        sliding_control_ = sliding_control;
        tactile_subscription_ = this->create_subscription<tactile_msgs::msg::TactileSignal>("/tactile_signals", 30, std::bind(&TactileUpdater::tactile_callback, this, _1));
        sliding_subscription_ = this->create_subscription<franka_msgs::msg::SlidingControl>("/sliding_control", 10, std::bind(&TactileUpdater::sliding_callback, this, _1));
    }

private:
    void sliding_callback(const franka_msgs::msg::SlidingControl::SharedPtr msg) const;
    void tactile_callback(const tactile_msgs::msg::TactileSignal::SharedPtr msg) const;

    float *reduced_average_, *sliding_control_;
    rclcpp::Subscription<franka_msgs::msg::SlidingControl>::SharedPtr sliding_subscription_;
    rclcpp::Subscription<tactile_msgs::msg::TactileSignal>::SharedPtr tactile_subscription_;
};