// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file spatem_ts_access.hpp
 * @brief Main SPATEM access header to include in ROS 2 projects
 */

#pragma once

// Messages
#include <etsi_its_spatem_ts_msgs/msg/spatem.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace etsi_its_spatem_ts_msgs {
    using namespace msg;
    namespace gm = geometry_msgs::msg;
}

// Implementation
#include <etsi_its_msgs_utils/impl/spatem/spatem_ts_access.h>