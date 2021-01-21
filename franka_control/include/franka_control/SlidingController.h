// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <functional>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace franka_control
{
    class SlidingController
    {
    public:
        SlidingController(const std::shared_ptr<franka::Model>);
        // ~SlidingController();

        void set_stiffness(const std::array<double, 6> &, const double);
        void set_sliding_parameter(const double, const std::array<double, 3> &, const std::array<double, 3> &);

        franka::CartesianVelocities sliding_control_callback(const franka::RobotState &, franka::Duration);
        franka::Torques force_control_callback(const franka::RobotState &, franka::Duration);
        franka::Torques touch_control_callback(const franka::RobotState &, franka::Duration);

    private:
        std::shared_ptr<franka::Model> model_ptr_;

        Eigen::MatrixXd stiffness_, damping_;
        Eigen::Affine3d initial_transform_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        Eigen::VectorXd initial_tau_ext_, tau_error_integral_;

        franka::RobotState initial_state_;

        std::array<double, 3> x_max_, dx_max_, dx_, sgn_, omega_, accel_time_, const_v_time_, time_max_;

        double force_, time_, desired_force_;

        const double FILTER_GAIN{0.001};
        const double K_p{1.0};
        const double K_i{2.0};
    };
} // namespace franka_control