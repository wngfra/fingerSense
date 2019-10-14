#pragma once

#include <array>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot);

franka::CartesianVelocities generateMotion(const std::array<double, 6> &command, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time);