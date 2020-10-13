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

#include <Eigen/Dense>

#include "franka_control/Action.h"

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
    Eigen::MatrixXd bound(6, 2);

    bound << -0.1, 0.1,
        0.0, 0.5,
        -0.2, 0.2,
        -M_PI_4 / 2.0, M_PI_4 / 2.0,
        -M_PI_4 / 2.0, M_PI_4 / 2.0,
        -M_PI_4 / 2.0, M_PI_4 / 2.0;

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

    std::cout << "weight: \n"
              << weights;

    for (int i = 0; i < 10; ++i)
    {
        double t = 0.1 * (double)i;
        auto values = action(t);
        print(values, title);
    }

    return 0;
}