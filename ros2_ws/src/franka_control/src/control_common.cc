#include <algorithm>
#include <array>
#include <math.h>
#include <iostream>

#include <franka/robot.h>

#include "franka_control_interface/control_common.h"

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
            if (v2 == 0) is_finished = true;
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
    robot.setCollisionBehavior(
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
        {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}