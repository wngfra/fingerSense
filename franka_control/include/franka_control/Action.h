// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <functional>
#include <tuple>
#include <vector>

namespace franka_control
{
    class Action
    {
    struct ActionBound
    {
        std::tuple<double, double> x, y, z;
        ActionBound(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) {
            x = std::make_tuple(x_min, x_max);
            y = std::make_tuple(y_min, y_max);
            z = std::make_tuple(z_min, z_max);
        }
    };

    public:
        Action(ActionBound);
        bool add_basis(std::function<const double&>);
        bool add_basis(std::vector<std::function<const double&(const double&)>>);
        const std::array<double, 6> operator()();
    private:
        ActionBound bound;
        std::vector<std::function<const double&(const double&)>> bases;
    };
} // namespace franka_control