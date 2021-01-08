// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <functional>
#include <memory>

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

        franka::CartesianVelocities sliding_control_callback(const franka::RobotState &, franka::Duration);
        franka::Torques force_control_callback(const franka::RobotState &, franka::Duration);
        franka::Torques touch_control_callback(const franka::RobotState &, franka::Duration);

        void set_stiffness(const std::array<double, 6> &, const double);
        void set_sliding_parameter(const double, const double, const double, const int);

        void reset_time();

    private:
        std::shared_ptr<franka::Model> model_ptr_;

        Eigen::MatrixXd stiffness_, damping_;
        Eigen::Affine3d initial_transform_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;

        franka::RobotState initial_state_;

        double x_max_, v_x_max_, accel_x_;
        double accel_time_, const_v_time_;
        double force_, force_error_integral_;
        double omega_;
        int cycle_max_;

        int cycle_count_ = 0;
        double time_ = 0.0;
    };
} // namespace franka_control