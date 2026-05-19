// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file denm_access.hpp
 * @brief Main DENM access header to include in ROS 2 projects
 */

#pragma once

// Messages
#include <etsi_its_denm_msgs/msg/denm.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace etsi_its_denm_msgs {
    using namespace msg;
    namespace gm = geometry_msgs::msg;
}

// Implementation
#include <etsi_its_msgs_utils/impl/denm/denm_access.h>
