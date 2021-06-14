// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <algorithm>
#include <stdio.h>
#include <math.h>
#include <numeric>

#include "franka_control/MotionController.h"

#define RATIO 0.2

namespace franka_control
{

    MotionController::MotionController(const std::shared_ptr<franka::Model> model_ptr)
    {
        model_ptr_ = model_ptr;
    }

    void MotionController::set_initial_orientation(const franka::RobotState &robot_state)
    {
        initial_state_ = robot_state;
        initial_transform_ = Eigen::Matrix4d::Map(initial_state_.O_T_EE_c.data());
        orientation_d_ = initial_transform_.linear();
    }

    void MotionController::set_stiffness(const std::array<double, 6> &stiffness_coefficient, const std::array<double, 6> &damping_coefficient)
    {
        // Compliance parameters
        Eigen::MatrixXd stiffness_matrix(6, 6), damping_matrix(6, 6);
        stiffness_matrix.setZero();
        damping_matrix.setZero();

        for (int i = 0; i < 6; ++i)
        {
            stiffness_matrix(i, i) = stiffness_coefficient[i];
            damping_matrix(i, i) = damping_coefficient[i] * sqrt(stiffness_coefficient[i]);
        }

        stiffness_ = stiffness_matrix;
        damping_ = damping_matrix;
    }

    void MotionController::set_parameter(const double force, const std::array<double, 3> &distance, const std::array<double, 3> &speed)
    {
        target_force_ = force;
        x_max_ = distance;
        dx_max_ = speed;

        for (int i = 0; i < 3; ++i)
        {
            if (x_max_[i] != 0.0)
            {
                double accx = RATIO * x_max_[i];
                sgn_[i] = x_max_[i] / std::abs(x_max_[i]);
                omega_[i] = 2.0 * M_PI * dx_max_[i] / x_max_[i];
                accel_time_[i] = M_PI / omega_[i];
                const_v_time_[i] = ((std::abs(x_max_[i]) - 2 * accx) / std::abs(dx_max_[i]));
                time_max_[i] = 2.0 * M_PI / omega_[i];
            }
        }

        time_ = 0.0;
    }

    franka::CartesianVelocities MotionController::LinearRelativeMotion(const franka::RobotState &robot_state, franka::Duration period)
    {
        time_ += period.toSec();

        if (time_ == 0.0)
        {
            initial_state_ = robot_state;
            initial_transform_ = Eigen::Matrix4d::Map(initial_state_.O_T_EE_c.data());
            position_d_ = initial_transform_.translation();

            // initialize controller variables
            dx_.fill(0.0);
            desired_force_ = 0.0;
            force_error_integral_ = 0.0;
            prev_force_error_ = 0.0;
        }
        for (int i = 0; i < 3; i++)
        {
            if (x_max_[i] != 0.0 && time_ <= time_max_[i])
            {

                dx_[i] = dx_max_[i] + dx_max_[i] * std::sin(omega_[i] * time_ - M_PI_2);
            }
        }

        franka::CartesianVelocities output = {{dx_[0], dx_[1], dx_[2], 0.0, 0.0, 0.0}};

        if (time_ > *std::max_element(time_max_.begin(), time_max_.end()))
        {
            output.motion_finished = true;
            time_ = 0.0;
        }

        return output;
    }

    franka::Torques MotionController::force_control_callback(const franka::RobotState &robot_state, franka::Duration period)
    {
        // get state variables
        std::array<double, 7> coriolis_array = model_ptr_->coriolis(robot_state);
        std::array<double, 42> jacobian_array = model_ptr_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        // compute error to desired equilibrium pose
        // position error
        std::array<double, 16> pose_d(robot_state.O_T_EE_d);
        // position_d_[0] = pose_d[12];
        position_d_[1] = pose_d[13];
        position_d_[2] = pose_d[14];

        desired_force_ = FILTER_GAIN * desired_force_ + (1 - FILTER_GAIN) * target_force_;
        // compute force error using the robot wrench sensors
        double force_error = desired_force_ + robot_state.O_F_ext_hat_K[2];
        double force_error_derivative = force_error - prev_force_error_;
        // update prev vars
        prev_force_error_ = force_error;

        force_error_integral_ += period.toSec() * force_error;
        position_d_[2] -= K_P * force_error + K_I * force_error_integral_ + K_D * force_error_derivative;

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
        // Spring damper system with damping
        tau_task << jacobian.transpose() * (-stiffness_ * error - damping_ * (jacobian * dq));
        tau_d << tau_task + coriolis;
        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

        return tau_d_array;
    }

    franka::Torques MotionController::dynamic_impedance_control(const franka::RobotState &robot_state, franka::Duration period)
    {
        time_ += period.toSec();

        if (time_ == 0.0)
        {
            initial_transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data());
            position_d_ = initial_transform_.translation();
            orientation_d_ = initial_transform_.linear();

            // Bias torque sensor
            std::array<double, 7> gravity_array = model_ptr_->gravity(robot_state);
            Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state_.tau_J.data());
            Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
            initial_tau_ext_ = initial_tau_measured - initial_gravity;
        }

        // get state variables
        std::array<double, 7> coriolis_array = model_ptr_->coriolis(robot_state);
        std::array<double, 42> jacobian_array = model_ptr_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
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
            // position_d_[0] += 1e-6;
            position_d_[2] -= 5e-5;
        }

        return output;
    }
} // namespace franka_control