// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <functional>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "common.h"

namespace franka_control
{
    class MotionController
    {
    public:
        MotionController(const std::shared_ptr<franka::Model>, std::shared_ptr<float> fp);
        // ~MotionController();

        void set_initial_orientation(const franka::RobotState &robot_state);
        void set_stiffness(const std::array<double, 6> &, const std::array<double, 6> &);
        void set_parameter(const double, const std::array<double, 3> &, const std::array<double, 3> &);

        franka::CartesianVelocities LinearRelativeMotion(const franka::RobotState &, franka::Duration);
        franka::Torques force_control_callback(const franka::RobotState &, franka::Duration);
        franka::Torques dynamic_impedance_control(const franka::RobotState &, franka::Duration);

    private:
        std::shared_ptr<franka::Model> model_ptr_;

        Eigen::MatrixXd stiffness_, damping_;
        Eigen::Affine3d initial_transform_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        Eigen::VectorXd initial_tau_ext_;

        franka::RobotState initial_state_;

        std::array<double, 3> x_max_, dx_max_, dx_, sgn_, omega_, accel_time_, const_v_time_, time_max_;
        // Store the float pointer to mean value of sensors
        std::shared_ptr<float> fp_;

        double target_force_, time_, desired_force_;
        double force_error_integral_, prev_force_error_;

        const double FILTER_GAIN{0.1};
        const double K_P{1.5e-3};
        const double K_I = 0.02 * K_P * K_P;
        const double K_D = 0.55 * K_P;
    };
} // namespace franka_control