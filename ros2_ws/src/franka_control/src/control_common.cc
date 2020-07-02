#include <algorithm>
#include <array>
#include <math.h>
#include <iostream>

#include <franka/robot.h>

#include "control_common.h"

franka::CartesianVelocities generateMotion(const std::array<double, 6> &v_command, franka::Duration period, double &time, const double time_limit, std::array<double, 6> &v_init)
{
    time += period.toSec();

    // Desired velocity
    std::array<double, 6> vd{};

    bool is_finished = false;

    for (int i = 0; i < 6; ++i)
    {
        double v1 = v_init[i];
        double v2 = v_command[i];

        if (v2 == v1)
        {
            if (v2 == 0)
                is_finished = true;
            vd[i] = v2;
        }
        else if (v2 > v1)
        {
            vd[i] = v1 + (v2 - v1) * std::sin(0.5 * M_PI * time / time_limit);
        }
        else
        {
            vd[i] = v2 - (v2 - v1) * std::cos(0.5 * M_PI * time / time_limit);
        }
    }

    franka::CartesianVelocities output(vd);

    if (is_finished)
        return franka::MotionFinished(output);
    else
        return output;
}

void setDefaultBehavior(franka::Robot &robot)
{
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}