// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "common.h"
#include "SlidingController.h"
#include "franka_interfaces/srv/sliding_control.hpp"

namespace franka_control
{
    class SlidingControlServer : public rclcpp::Node
    {
    public:
        SlidingControlServer(std::shared_ptr<franka::Robot> robot, std::shared_ptr<FrankaStates> franka_states, std::shared_ptr<float> fp) : Node("sliding_control_server")
        {
            robot_ = robot;
            franka_states_ = franka_states;

            setDefaultBehavior(*robot_);
            auto model_ptr = std::make_shared<franka::Model>(robot_->loadModel());
            controller_ = std::make_unique<SlidingController>(model_ptr, fp);

            service_ = this->create_service<franka_interfaces::srv::SlidingControl>("/sliding_control", std::bind(&SlidingControlServer::controlled_slide, this, std::placeholders::_1, std::placeholders::_2));

            is_touched = false;

            try
            {
                MotionGenerator mg(0.5, q_goal);
                robot_->control(mg);
            }
            catch (const franka::Exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "%s", e.what());
            }
        }

    private:
        void controlled_slide(const std::shared_ptr<franka_interfaces::srv::SlidingControl::Request>, std::shared_ptr<franka_interfaces::srv::SlidingControl::Response>);
        void update_franka_states(const franka::RobotState &robot_state) const;

        rclcpp::Service<franka_interfaces::srv::SlidingControl>::SharedPtr service_;

        std::array<double, 3> distance;
        std::array<double, 3> speed;
        double force;
        bool is_touched;

        const std::array<double, 7> q_goal = {{0.108882,-0.235987,0.0255725,-2.60724,0.0482073,2.3529,0.929222}};

        std::shared_ptr<FrankaStates> franka_states_;
        std::shared_ptr<franka::Robot> robot_;
        std::unique_ptr<SlidingController> controller_;
    };
} // namespace franka_control
