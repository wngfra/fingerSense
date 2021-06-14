// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <rclcpp/rclcpp.hpp>

#include "franka_control/MotionController.h"
#include "franka_interfaces/msg/robot_state.hpp"
#include "franka_interfaces/srv/sliding_control.hpp"

using namespace std::chrono_literals;

namespace franka_control
{
    class MotionControlServer : public rclcpp::Node
    {
    public:
        MotionControlServer(char* robot_ip);

    private:
        void timer_callback();

        void controlled_slide(const std::shared_ptr<franka_interfaces::srv::SlidingControl::Request>, std::shared_ptr<franka_interfaces::srv::SlidingControl::Response>);

        rclcpp::Service<franka_interfaces::srv::SlidingControl>::SharedPtr service_;
        rclcpp::Publisher<franka_interfaces::msg::RobotState>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::array<double, 3> distance;
        std::array<double, 3> speed;
        double force;
        bool is_touched;

        const std::array<double, 7> q_goal = {{M_PI / 12.0, 0.0, 0.0, -M_PI_2, 0.0, M_PI_2, M_PI / 4 + M_PI / 12.0}};

        franka::RobotState current_state_;
        std::unique_ptr<franka::Robot> robot_;
        std::shared_ptr<franka::Model> model_;
        std::unique_ptr<MotionController> controller_;
    };
} // namespace franka_control
