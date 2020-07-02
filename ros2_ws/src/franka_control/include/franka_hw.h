#pragma once

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace franka_hw {

class FrankaHW {
    public:
    FrankaHW();
    ~FrankaHW();
}

} // namespace franka_hw