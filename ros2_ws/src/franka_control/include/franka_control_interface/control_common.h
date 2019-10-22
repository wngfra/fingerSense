#pragma once

#include <array>
#include <condition_variable>
#include <mutex>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

franka::CartesianVelocities generateMotion(const std::array<double, 6> &commands, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time, std::array<double, 6> &vt);

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot &robot);