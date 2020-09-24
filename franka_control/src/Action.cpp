// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <stdio.h>

#include "franka_control/Action.h"

namespace franka_control
{
    Action::Action(const Eigen::MatrixXd &bound)
    {
        if (bound.cols() == 2 && bound.rows() == 6)
        {
            bound_ = bound;
        }
        else
        {
            printf("Incorrect action bound size! Unbounded action is enabled.\n");
        }
    }

    bool Action::addBases(std::function<double(const double &)> base, const Eigen::VectorXd &weight)
    {
        if (weight.size() != 6)
        {
            printf("Expected 6 weight coefficients, but %ld are given!\n", weight.size());
            return false;
        }

        bases_.push_back(base);

        weights_.conservativeResize(weight.rows(), weights_.cols() + 1);
        weights_.col(weights_.cols() - 1) = weight;
        return true;
    }

    bool Action::addBases(std::vector<std::function<double(const double &)>> bases, const Eigen::MatrixXd &weights)
    {
        if (weights.cols() != bases.size())
        {
            printf("Expected 6x%ld weight coefficients, but 6x%ld are given!\n", bases.size(), weights.cols());
            return false;
        }

        Eigen::Index insertIndex = weights_.cols();

        bases_.insert(bases_.end(), bases.begin(), bases.end());
        weights_.conservativeResize(weights.rows(), weights_.cols() + bases.size());
        weights_.block(0, insertIndex, weights.rows(), weights.cols()) = weights;

        return true;
    }

    Eigen::MatrixXd Action::getWeights() const
    {
        return weights_;
    }

    bool Action::setWeights(const Eigen::MatrixXd &w)
    {
        if (weights_.cols() == w.cols() && weights_.rows() == w.rows())
        {
            weights_ = w;

            return true;
        }

        return false;
    }

    bool Action::updateWeights(const Eigen::MatrixXd &dw)
    {
        if (weights_.cols() == dw.cols() && weights_.rows() == dw.rows())
        {
            weights_ += dw;

            return true;
        }

        return false;
    }

    std::array<double, 6> Action::operator()(const double &t) const
    {
        std::array<double, 6> output;
        output.fill(0);

        if (bases_.size() > 0)
        {
            Eigen::VectorXd values(bases_.size()), composite_action(6);

            for (int i = 0; i < bases_.size(); ++i)
            {
                values(i) = bases_[i](t);
            }

            composite_action = weights_ * values;

            if (bound_.size() > 0)
            {
                for (int j = 0; j < 6; ++j)
                {
                    if (bound_(j, 0) > composite_action(j))
                    {
                        composite_action(j) = bound_(j, 0);
                    }
                    else if (bound_(j, 1) < composite_action(j))
                    {
                        composite_action(j) = bound_(j, 1);
                    }
                }
            }

            Eigen::VectorXd::Map(&output[0], 6) = composite_action;
        }
        else
        {
            printf("No action bases added.\n");
        }

        return output;
    }
} // namespace franka_control