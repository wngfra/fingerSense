// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <algorithm>
#include <array>
#include <math.h>

#include "franka_control/SlidingController.h"

namespace franka_control
{

    SlidingController::SlidingController(const std::shared_ptr<franka::Model> model_ptr)
    {
        model_ptr_ = model_ptr;

        for (int i = 0; i < 3; ++i)
        {
            x_max_[i] = 0.0;
            dx_max_[i] = 0.0;
        }

        set_stiffness({{200, 200, 200, 20, 20, 20}}, 1.0);
        set_sliding_parameter(0.0, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}});
    }

    franka::CartesianVelocities SlidingController::sliding_control_callback(const franka::RobotState &robot_state, franka::Duration period)
    {
        time_ += period.toSec();

        if (time_ == 0.0)
        {
            force_error_integral_ = 0.0;
            initial_state_ = robot_state;
            initial_transform_ = Eigen::Matrix4d::Map(initial_state_.O_T_EE.data());
            position_d_ = initial_transform_.translation();
            orientation_d_ = initial_transform_.linear();
        }

        std::array<double, 3> dx;
        dx.fill(0.0);

        for (int i = 0; i < 3; ++i)
        {
            if (x_max_[i] > 0.0)
            {
                if (time_ <= accel_time_[i])
                {
                    dx[i] = dx_max_[i] * std::sin(omega_[i] * time_);
                }
                else if (time_ <= const_v_time_[i] + accel_time_[i])
                {
                    dx[i] = dx_max_[i];
                }
                else if (time_ <= time_max_[i])
                {
                    double t = time_ - const_v_time_[i];
                    dx[i] = dx_max_[i] * std::sin(omega_[i] * t);
                }
            }
        }

        franka::CartesianVelocities output = {{dx[0], dx[1], dx[2], 0.0, 0.0, 0.0}};

        if (time_ >= *std::max_element(time_max_.begin(), time_max_.end()))
        {
            output.motion_finished = true;
        }

        return output;
    }

    franka::Torques SlidingController::force_control_callback(const franka::RobotState &robot_state, franka::Duration period)
    {
        constexpr double k_p = 1e-5;
        constexpr double k_i = 1e-5;

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
        std::array<double, 16> pose_d(robot_state.O_T_EE_d);
        position_d_[0] = pose_d[12];
        position_d_[1] = pose_d[13];

        std::array<double, 6> wrench_ext(robot_state.O_F_ext_hat_K);
        // if (std::abs(wrench_ext[1]) >= 5.0)
        // {
        //     position_d_[1] = position(1);
        // }

        force_error_integral_ += period.toSec() * (-force_ - wrench_ext[2]);
        position_d_[2] += k_p * (-force_ - wrench_ext[2]) + k_i * force_error_integral_;

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
        if (std::abs(wrench_ext[2]) >= 10.0)
        {
            output.motion_finished = true;
        }
        else
        {
            position_d_[0] += 1e-5;
            position_d_[2] -= 5e-5;
        }

        return output;
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
            if (x_max_[i] > 0.0)
            {
                ddx_max_[i] = dx_max_[i] * dx_max_[i] / 1.5;
                omega_[i] = dx_max_[i] / ddx_max_[i];
                accel_time_[i] = M_PI_2 / omega_[i];
                const_v_time_[i] = (x_max_[i] - 2 * ddx_max_[i]) / dx_max_[i];
                time_max_[i] = 2 * accel_time_[i] + const_v_time_[i];
            }
            else
            {
                ddx_max_[i] = 0.0;
                omega_[i] = 0.0;
                accel_time_[i] = 0.0;
                const_v_time_[i] = 0.0;
                time_max_[i] = 0.0;
            }
        }

        time_ = 0.0;
    }

    void SlidingController::reset_time()
    {
        time_ = 0.0;
    }
} // namespace franka_control