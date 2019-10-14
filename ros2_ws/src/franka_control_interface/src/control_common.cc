#include "franka_control_interface/control_common.h"

#include <algorithm>
#include <array>
#include <math.h>
#include <iostream>

#include <Eigen/Dense>

#include <franka/model.h>
#include <franka/exception.h>
#include <franka/robot.h>

#define RESPONSE_COEFF 0.1

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

franka::CartesianVelocities generateMotion(const std::array<double, 6> &command, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time)
{
    std::array<double, 6> vs = {{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};

    if (command == vs)
    {
        franka::CartesianVelocities output(vs);
        return franka::MotionFinished(output);
    }
    else
    {
        time += period.toSec();
        if (time <= RESPONSE_COEFF)
            time = 0.0;
        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

        // Current Cartesian Velocity
        Eigen::Matrix<double, 6, 1> v0 = jacobian * dq;
        std::array<double, 6> v0_array{};
        Eigen::VectorXd::Map(&v0_array[0], 6) = v0;

        for (int i = 0; i < 6; ++i)
        {
            vs[i] = 0;
            std::cout << v0[i] << std::endl;
        }

        return franka::CartesianVelocities(vs);
    }
}