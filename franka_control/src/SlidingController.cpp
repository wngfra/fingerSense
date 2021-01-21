// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <algorithm>
#include <array>
#include <math.h>
#include <stdio.h>

#include "franka_control/SlidingController.h"

#define DDX_MAX 1.0

namespace franka_control
{

    SlidingController::SlidingController(const std::shared_ptr<franka::Model> model_ptr)
    {
        model_ptr_ = model_ptr;

        set_stiffness({{1000, 200, 200, 20, 20, 20}}, 1.0);
        set_sliding_parameter(0.0, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}});
    }

    void SlidingController::set_stiffness(const std::array<double, 6> &stiffness_coefficient, const double damping_coefficient)
    {
        // Compliance parameters
        Eigen::MatrixXd stiffness_matrix(6, 6), damping_matrix(6, 6);
        stiffness_matrix.setZero();
        damping_matrix.setZero();

        for (int i = 0; i < 6; ++i)
        {
            stiffness_matrix(i, i) = stiffness_coefficient[i];
            damping_matrix(i, i) = damping_coefficient * sqrt(stiffness_coefficient[i]);
        }

        stiffness_ = stiffness_matrix;
        damping_ = damping_matrix;
    }

    void SlidingController::set_sliding_parameter(const double force, const std::array<double, 3> &distance, const std::array<double, 3> &speed)
    {
        force_ = force;
        x_max_ = distance;
        dx_max_ = speed;

        for (int i = 0; i < 3; ++i)
        {
            if (x_max_[i] != 0.0)
            {
                sgn_[i] = x_max_[i] / std::abs(x_max_[i]);
                omega_[i] = 2 * DDX_MAX / std::abs(dx_max_[i]);
                accel_time_[i] = M_PI / omega_[i];
                const_v_time_[i] = (std::abs(x_max_[i]) - 2 * DDX_MAX * M_PI / (omega_[i] * omega_[i])) / std::abs(dx_max_[i]);
                time_max_[i] = 2 * accel_time_[i] + const_v_time_[i];
            }
        }

        time_ = 0.0;
    }

    franka::CartesianVelocities SlidingController::sliding_control_callback(const franka::RobotState &robot_state, franka::Duration period)
    {
        time_ += period.toSec();

        if (time_ == 0.0)
        {
            initial_state_ = robot_state;
            initial_transform_ = Eigen::Matrix4d::Map(initial_state_.O_T_EE.data());
            position_d_ = initial_transform_.translation();
            orientation_d_ = initial_transform_.linear();
            dx_.fill(0.0);

            // init integrator
            tau_error_integral_ = Eigen::VectorXd::Zero(7);

            // Bias torque sensor
            std::array<double, 7> gravity_array = model_ptr_->gravity(robot_state);
            Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state_.tau_J.data());
            Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
            initial_tau_ext_ = initial_tau_measured - initial_gravity;
        }

        for (int i = 0; i < 3; i++)
        {
            if (x_max_[i] != 0.0)
            {
                if (time_ <= accel_time_[i])
                {
                    dx_[i] = sgn_[i] * (DDX_MAX / omega_[i] - DDX_MAX / omega_[i] * std::cos(omega_[i] * time_));
                }
                else if (time_ > const_v_time_[i] + accel_time_[i] && time_ <= time_max_[i])
                {
                    dx_[i] = sgn_[i] * (DDX_MAX / omega_[i] - DDX_MAX / omega_[i] * std::cos(omega_[i] * (time_ - const_v_time_[i])));
                }
            }
        }

        franka::CartesianVelocities output = {{dx_[0], dx_[1], dx_[2], 0.0, 0.0, 0.0}};

        if (time_ >= *std::max_element(time_max_.begin(), time_max_.end()))
        {
            output.motion_finished = true;
            time_ = 0.0;
        }

        return output;
    }

    franka::Torques SlidingController::force_control_callback(const franka::RobotState &robot_state, franka::Duration period)
    {
        // get state variables
        std::array<double, 7> gravity_array = model_ptr_->gravity(robot_state);
        std::array<double, 42> jacobian_array = model_ptr_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());

        Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);

        // REVIEW: smooth force control not working
        desired_force_torque.setZero();
        desired_force_torque(2) = -desired_force_;
        tau_ext << tau_measured + gravity - initial_tau_ext_;
        tau_d << jacobian.transpose() * desired_force_torque;
        tau_error_integral_ += period.toSec() * (tau_d - tau_ext);

        // FF + PI control
        tau_cmd << tau_d + K_p * (tau_d - tau_ext) + K_i * tau_error_integral_;

        // Smoothly update the mass to reach the desired target value
        desired_force_ = FILTER_GAIN * desired_force_ + (1 - FILTER_GAIN) * force_;

        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

        return tau_d_array;
    }

    franka::Torques SlidingController::touch_control_callback(const franka::RobotState &robot_state, franka::Duration period)
    {
        time_ += period.toSec();

        if (time_ == 0.0)
        {
            initial_transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
            position_d_ = initial_transform_.translation();
            orientation_d_ = initial_transform_.linear();
        }

        // get state variables
        std::array<double, 7> coriolis_array = model_ptr_->coriolis(robot_state);
        std::array<double, 42> jacobian_array = model_ptr_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());
        // compute error to desired equilibrium pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d_;
        // orientation error
        // "difference" quaternion
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.linear() * error.tail(3);
        // compute control
        Eigen::VectorXd tau_task(7), tau_d(7);
        // Spring damper system with damping ratio=1
        tau_task << jacobian.transpose() * (-stiffness_ * error - damping_ * (jacobian * dq));
        tau_d << tau_task + coriolis;
        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

        franka::Torques output(tau_d_array);

        std::array<double, 6> wrench_ext(robot_state.O_F_ext_hat_K);
        if (std::abs(wrench_ext[2]) >= 8.0)
        {
            output.motion_finished = true;
            time_ = 0.0;
        }
        else
        {
            position_d_[0] += 1e-6;
            position_d_[2] -= 5e-5;
        }

        return output;
    }
} // namespace franka_control