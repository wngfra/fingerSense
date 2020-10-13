// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/duration.h>
#include <franka/robot.h>

namespace franka_control
{
    class SlidingControl
    {
    public:
        SlidingControl();
        // ~SlidingControl();

        franka::CartesianVelocities operator()(const franka::RobotState&, franka::Duration);
        void set_parameter(const double, const double, const double, const int);

    private:
        double x_max_, z_max_, v_x_max_, accel_x_;
        double accel_time_, const_v_time_;
        double omega_;
        int cycle_max_;

        int cycle_count_ = 0;
        double time_ = 0.0;
    };
} // namespace franka_control