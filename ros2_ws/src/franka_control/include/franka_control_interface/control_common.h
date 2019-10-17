#pragma once

#include <array>
#include <mutex>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

struct FrankaCommand
{
    std::mutex mutex;
    std::array<double, 6> command;

    FrankaCommand() : command{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}} {}

    bool is_zero()
    {
        for (auto &c : command)
        {
            if (c != 0.0)
                return false;
        }
        return true;
    }

    bool is_terminated()
    {
        for (auto &c : command)
        {
            if (c != -1.0)
                return false;
        }
        return true;
    }

    std::array<double, 6> fetch()
    {
        std::lock_guard<std::mutex> lock(mutex);
        std::array<double, 6> tmp = std::move(command);
        command = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return tmp;
    }

    void set(const std::array<double, 6> &new_command)
    {
        std::lock_guard<std::mutex> lock(mutex);
        command = std::move(new_command);
    }
};

franka::CartesianVelocities generateMotion(const std::array<double, 6> &command, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time);

template <typename T>
void print(const std::string &title, const T &as);

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot &robot);