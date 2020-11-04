// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <math.h>
#include <stdio.h>

#include "franka_control/SlidingControl.h"

namespace franka_control
{

    SlidingControl::SlidingControl(const std::shared_ptr<franka::Model> model_ptr)
    {
        model_ptr_ = model_ptr;

        x_max_ = 0.0;
        v_x_max_ = 0.0;

        set_stiffness({{200, 200, 200, 20, 20, 20}}, 1.0);
        set_sliding_parameter(0.0, 0.0, 0.0, 0);
    }

    franka::CartesianVelocities SlidingControl::operator()(const franka::RobotState &robot_state, franka::Duration period)
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

        double v_x = 0.0;
        double v_y = 0.0;
        const double total_time = 4 * accel_time_ + 2 * const_v_time_;

        if (time_ <= accel_time_)
        {
            v_x = v_x_max_ * std::sin(omega_ * time_);
        }
        else if (time_ <= const_v_time_ + accel_time_)
        {
            v_x = v_x_max_;
        }
        else if (time_ <= const_v_time_ + 3 * accel_time_)
        {
            double t = time_ - const_v_time_;
            v_x = v_x_max_ * std::sin(omega_ * t);
        }
        else if (time_ <= total_time - accel_time_)
        {
            v_x = -v_x_max_;
        }
        else
        {
            double t = time_ - 2 * const_v_time_;
            v_x = v_x_max_ * std::sin(omega_ * t);
        }

        // v_y = 0.05 * std::sin(32 * M_PI / total_time * time_);

        if (time_ >= total_time)
        {
            time_ -= total_time;
            cycle_count_ += 1;
        }

        franka::CartesianVelocities output = {{v_x, v_y, 0.0, 0.0, 0.0, 0.0}};

        if (x_max_ <= accel_x_ || cycle_count_ >= cycle_max_)
        {
            output.motion_finished = true;
        }

        return output;
    }

    franka::Torques SlidingControl::force_control_callback(const franka::RobotState &robot_state, franka::Duration period)
    {
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

        std::array<double, 6> wrench_ext(robot_state.O_F_ext_hat_K);
        if (std::abs(wrench_ext[1]) >= 5.0)
        {
            position_d_[1] = position(1);
        }

        force_error_integral_ += period.toSec() * (-force_ - wrench_ext[2]);
        position_d_[2] += 2e-6 * (-force_ - wrench_ext[2]) + 1e-6 * force_error_integral_;

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

    franka::Torques SlidingControl::touch_control_callback(const franka::RobotState &robot_state, franka::Duration period)
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

    void SlidingControl::set_stiffness(const std::array<double, 6> &stiffness_coefficient, const double damping_coefficient)
    {
        // Compliance parameters
        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        damping.setZero();

        stiffness(0, 0) = stiffness_coefficient[0];
        stiffness(1, 1) = stiffness_coefficient[1];
        stiffness(2, 2) = stiffness_coefficient[2];
        stiffness(3, 3) = stiffness_coefficient[3];
        stiffness(4, 4) = stiffness_coefficient[4];
        stiffness(5, 5) = stiffness_coefficient[5];

        damping(0, 0) = damping_coefficient * sqrt(stiffness_coefficient[0]);
        damping(1, 1) = damping_coefficient * sqrt(stiffness_coefficient[1]);
        damping(2, 2) = damping_coefficient * sqrt(stiffness_coefficient[2]);
        damping(3, 3) = damping_coefficient * sqrt(stiffness_coefficient[3]);
        damping(4, 4) = damping_coefficient * sqrt(stiffness_coefficient[4]);
        damping(5, 5) = damping_coefficient * sqrt(stiffness_coefficient[5]);

        stiffness_ = stiffness;
        damping_ = damping;
    }

    void SlidingControl::set_sliding_parameter(const double distance, const double force, const double speed, const int cycle_max)
    {
        x_max_ = distance;
        v_x_max_ = speed;
        cycle_max_ = cycle_max;
        force_ = force;

        accel_x_ = v_x_max_ * v_x_max_ / 1.5; // limit max ddx
        omega_ = v_x_max_ / accel_x_;
        accel_time_ = M_PI_2 / omega_;
        const_v_time_ = (x_max_ - 2 * accel_x_) / v_x_max_;
        time_ = 0.0;
        cycle_count_ = 0;
    }

    void SlidingControl::reset_time()
    {
        time_ = 0.0;
    }
} // namespace franka_control