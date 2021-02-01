// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "common.h"
#include "franka_interfaces/msg/robot_state.hpp"

using namespace std::chrono_literals;

namespace franka_control
{
    class FrankaStatePublisher : public rclcpp::Node
    {
    public:
        FrankaStatePublisher(std::shared_ptr<FrankaStates> franka_states) : Node("franka_state_publisher")
        {
            franka_states_ = franka_states;

            publisher_ = this->create_publisher<franka_interfaces::msg::RobotState>("franka_state", 10);
            timer_ = this->create_wall_timer(1ms, std::bind(&FrankaStatePublisher::timer_callback, this));
        }

    private:
        void timer_callback();

        std::shared_ptr<FrankaStates> franka_states_;

        rclcpp::Publisher<franka_interfaces::msg::RobotState>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
} // namespace franka_control