// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <functional>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

namespace franka_control
{
    class Action
    {
    public:
        Action(const Eigen::MatrixXd &);
        bool addBases(std::function<double(const double &)>, const Eigen::VectorXd &);
        bool addBases(std::vector<std::function<double(const double &)>>, const Eigen::MatrixXd &);
        Eigen::MatrixXd getWeights() const;
        bool isBounded() const;
        bool setWeights(const Eigen::MatrixXd &);
        bool updateWeights(const Eigen::MatrixXd &);
        std::array<double, 6> operator()(const double &) const;

    private:
        bool is_bounded;

        Eigen::MatrixXd bound_;
        Eigen::MatrixXd weights_;

        std::vector<std::function<double(const double &)>> bases_;
    };
} // namespace franka_control