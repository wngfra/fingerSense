// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <exception>

#include "franka_control/SlidingControlServer.h"

namespace franka_control
{
    void SlidingControlServer::controlled_slide(const std::shared_ptr<franka_interfaces::srv::SlidingControl::Request> request, std::shared_ptr<franka_interfaces::srv::SlidingControl::Response> response)
    {
        try
        {
            if (!is_touched)
            {
                robot_->control(
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques {
                        return controller_->touch_control_callback(robot_state, period);
                    });

                is_touched = true;
                RCLCPP_INFO(this->get_logger(), "Touched the platform.");
            }

            force = request->force;
            distance = request->distance;
            speed = request->speed;

            controller_->set_sliding_parameter(force, distance, speed);
            if (force > 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "Sliding force: %f, distance: (%f, %f, %f), speed: (%f, %f, %f).", force, distance[0], distance[1], distance[2], speed[0], speed[1], speed[2]);
                robot_->control(
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques {
                        return controller_->force_control_callback(robot_state, period);
                    },
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        return controller_->sliding_control_callback(robot_state, period);
                    });
            }
            else if (force == 0.0)
            {
                robot_->control(
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        return controller_->sliding_control_callback(robot_state, period);
                    });
            }
            else if (force < 0.0)
            {
                MotionGenerator mg(0.5, q_goal);
                robot_->control(mg);
            }

            response->success = true;
            response->recovered = false;
        }
        catch (const std::exception &e)
        {
            robot_->automaticErrorRecovery();

            RCLCPP_WARN(this->get_logger(), "%s! Automatic recovery attempted.", e.what());
            response->success = false;
            response->recovered = true;
        }
    }
} // namespace franka_control
