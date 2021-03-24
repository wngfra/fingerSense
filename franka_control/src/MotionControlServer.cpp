// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <exception>

#include "franka_control/MotionControlServer.h"

namespace franka_control
{
    void MotionControlServer::controlled_slide(const std::shared_ptr<franka_interfaces::srv::SlidingControl::Request> request, std::shared_ptr<franka_interfaces::srv::SlidingControl::Response> response)
    {
        try
        {
            force = request->force;
            distance = request->distance;
            speed = request->speed;

            std::array<double, 6> damping_coefficient;
            damping_coefficient.fill(1.0);

            controller_->set_parameter(force, distance, speed);
            if (force > 0.0)
            {
                if (!is_touched)
                {
                    controller_->set_stiffness({{1000, 1000, 300, 300, 300, 300}}, damping_coefficient);
                    robot_->control(    
                        [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques {
                            update_franka_states(robot_state);
                            return controller_->dynamic_impedance_control(robot_state, period);
                        });

                    is_touched = true;
                    RCLCPP_INFO(this->get_logger(), "Touched the platform.");

                    controller_->set_stiffness({{3000, 3000, 500, 300, 100, 300}}, damping_coefficient);
                }

                RCLCPP_INFO(this->get_logger(), "Sliding force: %f, distance: (%f, %f, %f), speed: (%f, %f, %f).", force, distance[0], distance[1], distance[2], speed[0], speed[1], speed[2]);
                robot_->control(
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques {
                        return controller_->force_control_callback(robot_state, period);
                    },
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        update_franka_states(robot_state);
                        return controller_->LinearRelativeMotion(robot_state, period);
                    });
            }
            else if (force == 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "Moving distance: (%f, %f, %f), speed: (%f, %f, %f).", distance[0], distance[1], distance[2], speed[0], speed[1], speed[2]);
                robot_->control(
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        update_franka_states(robot_state);
                        return controller_->LinearRelativeMotion(robot_state, period);
                    });
                is_touched = false;
            }
            else if (force < 0.0)
            {
                MotionGenerator mg(0.5, q_goal);
                robot_->control(mg);
                is_touched = false;
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

    void MotionControlServer::update_franka_states(const franka::RobotState &robot_state) const
    {
        franka_states_->external_wrench = robot_state.O_F_ext_hat_K;
        franka_states_->end_effector_pose = robot_state.O_T_EE;
    }

} // namespace franka_control
