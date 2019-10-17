#include "franka_control_interface/control_common.h"

#include <algorithm>
#include <array>
#include <math.h>
#include <iostream>

#include <Eigen/Dense>

#include <franka/model.h>
#include <franka/exception.h>
#include <franka/robot.h>

#define RESPONSE_TIME 1

franka::CartesianVelocities generateMotion(const std::array<double, 6> &command, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time)
{
    time += period.toSec();

    // Compute current Cartesian velocities
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Matrix<double, 6, 1> v0 = jacobian * dq;

    // Convert from Eigen
    std::array<double, 6> v0_array{};
    Eigen::VectorXd::Map(&v0_array[0], 6) = v0;

    if (time >= RESPONSE_TIME)
    {
        franka::CartesianVelocities velocity_desired(v0_array);
        return franka::MotionFinished(velocity_desired);
    }
    else
    {
        for (int i = 0; i < 6; ++i)
        {
            double v1 = v0_array[i];
            double v2 = command[i];
            v0_array[i] = (v2 - v1) / 2 * std::sin(M_PI / (RESPONSE_TIME)*time - M_PI_2) + (v1 + v2) / 2;
        }

        return franka::CartesianVelocities(v0_array);
    }
}

template <typename T>
void print(const std::string &title, const T &as)
{
    std::cout << title << " ";
    for (auto &a : as)
    {
        std::cout << a << " ";
    }
    std::cout << std::endl;
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