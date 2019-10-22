#pragma once

#include <array>
#include <condition_variable>
#include <mutex>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

struct FrankaCommand
{
    bool is_set = false;

    std::array<double, 6> command, last_command;

    std::mutex mutex;
    std::condition_variable cv;

    FrankaCommand() : last_command{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}} {}

    // Copy commands to a size 12 std::array
    std::array<double, 12> fetch()
    {
        std::unique_lock<std::mutex> lock(mutex);

        cv.wait(lock, [&]() { return is_set; });

        is_set = false;

        std::array<double, 12> commands;
        std::copy(command.begin(), command.end(), commands.begin());
        std::copy(last_command.begin(), last_command.end(), commands.begin() + 6);

        lock.unlock();

        return commands;
    }

    // Set command by std::array
    void set_new(const std::array<double, 6> &new_command)
    {
        std::unique_lock<std::mutex> lock(mutex);
        
        is_set = true;
        
        last_command = std::move(command);
        command = std::move(new_command);

        for (auto &c: command)
            std::cout << c;

        lock.unlock();
        cv.notify_all();
    }
};

franka::CartesianVelocities generateMotion(const std::array<double, 12> &commands, const franka::Model &model, franka::Duration period, const franka::RobotState &robot_state, double &time);

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot &robot);