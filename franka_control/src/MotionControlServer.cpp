// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <exception>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "franka_control/common.h"
#include "franka_control/MotionControlServer.h"

namespace franka_control
{
    MotionControlServer::MotionControlServer(char *robot_ip) : Node("sliding_control_server")
    {
        robot_ = std::make_unique<franka::Robot>(robot_ip, getRealtimeConfig());
        setDefaultBehavior(*robot_);
        RCLCPP_INFO(this->get_logger(), "Connected to robot@%s", robot_ip);

        model_ = std::make_shared<franka::Model>(robot_->loadModel());
        controller_ = std::make_unique<MotionController>(model_);

        try
        {
            MotionGenerator mg(0.5, q_goal);
            robot_->control(mg);
        }
        catch (const franka::Exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "%s", e.what());
        }

        service_ = this->create_service<franka_interfaces::srv::SlidingControl>("/sliding_control", std::bind(&MotionControlServer::controlled_slide, this, std::placeholders::_1, std::placeholders::_2));

        publisher_ = this->create_publisher<franka_interfaces::msg::RobotState>("franka_state", 10);
        timer_ = this->create_wall_timer(1ms, std::bind(&MotionControlServer::timer_callback, this));
    }

    void MotionControlServer::timer_callback()
    {
        auto msg = franka_interfaces::msg::RobotState();
        msg.header.frame_id = "base";
        msg.header.stamp = this->get_clock()->now();

        msg.external_wrench = current_state_.O_F_ext_hat_K;

        Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_state_.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        Eigen::VectorXd::Map(&msg.position[0], 3) = position;
        Eigen::VectorXd::Map(&msg.orientation[0], 4) = orientation.coeffs();

        std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, current_state_);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(current_state_.dq.data());
        Eigen::VectorXd::Map(&msg.velocity[0], 6) = jacobian * dq;

        publisher_->publish(msg);
    }

    void MotionControlServer::controlled_slide(const std::shared_ptr<franka_interfaces::srv::SlidingControl::Request> request, std::shared_ptr<franka_interfaces::srv::SlidingControl::Response> response)
    {
        bool success;
        int control_type(0);
        try
        {
            force = request->force;
            distance = request->distance;
            speed = request->speed;

            std::array<double, 6> damping_coefficient;
            damping_coefficient.fill(0.8);

            controller_->set_parameter(force, distance, speed);
            if (force < 0.0)
            {
                control_type = 1;
                MotionGenerator mg(0.5, q_goal);
                robot_->control(mg);
            }

            else if (force == 0.0)
            {
                control_type = 2;
                controller_->set_stiffness({{1000, 1000, 200, 300, 300, 300}}, damping_coefficient);
                robot_->control(
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques
                    {
                        current_state_ = robot_state;
                        return controller_->dynamic_impedance_control(robot_state, period);
                    });

                RCLCPP_INFO(this->get_logger(), "Touched the platform.");

                controller_->set_initial_orientation(robot_->readOnce());
                controller_->set_stiffness({{1000, 3000, 400, 300, 300, 300}}, damping_coefficient);  
            }
            else if (force > 0.0)
            {
                control_type = 3;
                RCLCPP_INFO(this->get_logger(), "Sliding force: %f, distance: (%f, %f, %f), speed: (%f, %f, %f).", force, distance[0], distance[1], distance[2], speed[0], speed[1], speed[2]);
                robot_->control(
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques
                    {
                        current_state_ = robot_state;
                        return controller_->force_control_callback(robot_state, period);
                    },
                    [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::CartesianVelocities
                    {
                        return controller_->LinearRelativeMotion(robot_state, period);
                    });
            }

            success = true;
        }
        catch (const std::exception &e)
        {
            robot_->automaticErrorRecovery();

            RCLCPP_WARN(this->get_logger(), "%s! Automatic recovery attempted.", e.what());
            success = false;
            control_type = -1;
        }

        response->success = success;
        response->type = control_type;
    }
} // namespace franka_control
