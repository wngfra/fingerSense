// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <functional>
#include <memory>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace franka_control
{
    class SlidingControl
    {
    public:
        SlidingControl(const std::shared_ptr<franka::Model>);
        // ~SlidingControl();

        franka::CartesianVelocities operator()(const franka::RobotState &, franka::Duration);
        franka::Torques force_control_callback(const franka::RobotState &, franka::Duration);
        void set_parameter(const double, const double, const double, const int);

    private:
        std::shared_ptr<franka::Model> model_ptr_;

        const double translational_stiffness{150.0};
        const double rotational_stiffness{10.0};

        Eigen::MatrixXd stiffness_, damping_;
        Eigen::Affine3d initial_transform_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;

        franka::RobotState initial_state_;

        double x_max_, v_x_max_, accel_x_;
        double accel_time_, const_v_time_;
        double force_;
        double omega_;
        int cycle_max_;

        int cycle_count_ = 0;
        double time_ = 0.0;
    };
} // namespace franka_control