// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file mcm_access.hpp
 * @brief Main MCM access header to include in ROS 2 projects
 */

#pragma once

// Messages
#include <etsi_its_mcm_uulm_msgs/msg/mcm.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace etsi_its_mcm_uulm_msgs {
using namespace msg;
namespace gm = geometry_msgs::msg;
}  // namespace etsi_its_mcm_uulm_msgs

// Implementation
#include <etsi_its_msgs_utils/impl/mcm/mcm_access.h>