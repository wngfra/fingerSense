// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <math.h>
#include <stdio.h>

#include "franka_control/SlidingControl.h"

namespace franka_control
{

    SlidingControl::SlidingControl()
    {
        x_max_ = 0.0;
        v_x_max_ = 0.0;

        set_parameter(0.0, 0.0, 0);
    }

    franka::CartesianVelocities SlidingControl::operator()(const franka::RobotState &robot_state, franka::Duration period)
    {
        time_ += period.toSec();

        double v_x = 0.0;

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
        else if (time_ <= 2 * const_v_time_ + 3 * accel_time_)
        {
            v_x = -v_x_max_;
        }
        else if (time_ <= 2 * const_v_time_ + 4 * accel_time_)
        {
            double t = time_ - 2 * const_v_time_;
            v_x = v_x_max_ * std::sin(omega_ * t);
        }

        franka::CartesianVelocities output = {{v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};

        if (time_ >= 2 * const_v_time_ + 4 * accel_time_)
        {
            time_ -= 2 * const_v_time_ + 4 * accel_time_;
            cycle_count_ += 1;
        }

        if (x_max_ <= accel_x_ || cycle_count_ >= cycle_max_)
        {
            output.motion_finished = true;
        }

        return output;
    }

    void SlidingControl::set_parameter(const double distance, const double speed, const int cycle_max)
    {
        x_max_ = distance;
        v_x_max_ = speed;
        cycle_max_ = cycle_max;

        accel_x_ = v_x_max_ * v_x_max_ / 1.5; // limit max ddx
        omega_ = v_x_max_ / accel_x_;
        accel_time_ = M_PI_2 / omega_;
        const_v_time_ = (x_max_ - 2 * accel_x_) / v_x_max_;
        time_ = 0.0;
        cycle_count_ = 0;
    }
} // namespace franka_control