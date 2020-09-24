// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <iostream>
#include <math.h>
#include <memory>
#include <numeric>
#include <stdlib.h>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <rclcpp/rclcpp.hpp>

#include "franka_control/Action.h"
#include "tactile_msgs/srv/change_state.hpp"
#include "franka_control/common.h"
#include "franka_control/TactileUpdater.h"

using namespace std::chrono_literals;

void print(const std::array<double, 6> &arr, std::string &title)
{
    std::cout << title << ": ";
    for (auto &v : arr)
    {
        std::cout << v << " ";
    }
    std::cout << std::endl;
}

int main(int argc, char **argv)
{

    int n = 10;
    Eigen::MatrixXd bound;
    Eigen::MatrixXd weights(6, n);
    std::vector<std::function<double(const double &)>> bases;
    franka_control::Action action(bound);

    for (int i = 0; i < n; ++i)
    {
        weights.col(i) = Eigen::VectorXd::Random(6);
        bases.push_back([&](const double &t) -> double {
            return sin((double)i * t);
        });
    }

    bool res = action.addBases(bases, weights);
    std::string title = "Action";
    
    for (int i = 0; i < 1000; ++i){
        auto values = action((double)i * 0.01);
        print(values, title);
    }

    return 0;
}