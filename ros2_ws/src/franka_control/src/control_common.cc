#include <algorithm>
#include <array>
#include <math.h>
#include <iostream>

#include <Eigen/Dense>

#include <franka/model.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "franka_control_interface/control_common.h"

#define RESPONSE_TIME 1.0

franka::CartesianVelocities generateMotion(const std::array<double, 6> &vc, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time, std::array<double, 6> &vt)
{
    if (time >= RESPONSE_TIME)
    {
        time = 0.0;
    }

    // Compute the initial velocity
    if (time == 0.0)
    {
        // Compute current Cartesian velocities
        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Matrix<double, 6, 1> v0 = jacobian * dq;

        // Convert from Eigen
        std::array<double, 6> v0_array{};
        Eigen::VectorXd::Map(&v0_array[0], 6) = v0;
        vt = v0_array;
    }

    time += period.toSec();

    // Desired velocity
    std::array<double, 6> vd{};

    bool is_finished = false;

    for (int i = 0; i < 6; ++i)
    {
        double v1 = vt[i];
        double v2 = vc[i];

        if (v2 == v1)
        {
            vd[i] = v2;
        }
        else if (v2 > v1)
        {
            vd[i] = v1 + (v2 - v1) * std::sin(0.5 * M_PI * time / RESPONSE_TIME);
        }
        else
        {
            vd[i] = v2 - (v2 - v1) * std::cos(0.5 * M_PI * time / RESPONSE_TIME);
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
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}