// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "franka_interfaces/msg/robot_state.hpp"

using namespace std::chrono_literals;

namespace franka_control
{
    class FrankaStatePublisher : public rclcpp::Node
    {
    public:
        FrankaStatePublisher(std::shared_ptr<bool> bMotionFinished, std::shared_ptr<std::array<double, 6>> O_F_ext_hat_K, std::shared_ptr<std::array<double, 3>> position, std::shared_ptr<std::array<double, 4>> quaternion) : Node("franka_state_publisher")
        {
            bMotionFinished_ = bMotionFinished;
            O_F_ext_hat_K_ = O_F_ext_hat_K;
            position_ = position;
            quaternion_ = quaternion;

            publisher_ = this->create_publisher<franka_interfaces::msg::RobotState>("franka_state", 10);
            timer_ = this->create_wall_timer(1ms, std::bind(&FrankaStatePublisher::timer_callback, this));
        }

    private:
        void timer_callback();

        std::shared_ptr<bool> bMotionFinished_;
        std::shared_ptr<std::array<double, 6>> O_F_ext_hat_K_;
        std::shared_ptr<std::array<double, 3>> position_;
        std::shared_ptr<std::array<double, 4>> quaternion_;

        rclcpp::Publisher<franka_interfaces::msg::RobotState>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
} // namespace franka_control