// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka_control/TactileUpdater.h"

void TactileUpdater::tactile_callback(const tactile_msgs::msg::TactileSignal::SharedPtr msg) const
{
    // Computes the average tactile response
    auto buffer = msg->data;
    float reduced_average = std::accumulate(buffer.begin(), buffer.end(), 0.0, [&](float reduced_average, int e) {
        return reduced_average + (float)e / (float)buffer.size();
    });

    *reduced_average_ = reduced_average;
}

void TactileUpdater::sliding_callback(const franka_msgs::msg::SlidingControl::SharedPtr msg) const
{
    *sliding_control_ = msg->pressure;
    *(sliding_control_ + 1) = msg->speed;
}