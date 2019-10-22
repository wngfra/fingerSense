#include <algorithm>
#include <array>
#include <math.h>
#include <iostream>

#include <Eigen/Dense>

#include <franka/model.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "franka_control_interface/control_common.h"

#define RESPONSE_TIME 0.152

franka::CartesianVelocities generateMotion(const std::array<double, 12> &commands, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time)
{
    std::array<double, 6> vt;

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

    franka::CartesianVelocities output(vd);

    if (time >= RESPONSE_TIME)
    {
        return franka::MotionFinished(output);
    }
    else
    {
        double vc, vp;
        for (int i = 0; i < 6; ++i)
        {
            // Current velocity command
            vc = commands[i];
            // Last velocity command
            vp = vt[i];
            vd[i] = vc * std::sin(M_PI * time / RESPONSE_TIME);
            /*
            if (vc > vp)
            {
                vd[i] = vp + (vc - vp) * std::sin(0.5 * M_PI * time / RESPONSE_TIME);
            }
            else if (vc == vp)
            {
                vd[i] = vc;
            }
            else if (vc < vp)
            {
                vd[i] = vp + (vc - vp) * std::sin(0.5 * M_PI * (time - RESPONSE_TIME) / RESPONSE_TIME);
            }
            */
        }

        output = franka::CartesianVelocities(vd);

        return output;
    }
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

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}