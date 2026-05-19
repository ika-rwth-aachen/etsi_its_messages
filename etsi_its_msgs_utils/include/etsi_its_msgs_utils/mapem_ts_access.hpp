// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file mapem_ts_access.hpp
 * @brief Main MAPEM access header to include in ROS 2 projects
 */

#pragma once

// Messages
#include <etsi_its_mapem_ts_msgs/msg/mapem.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace etsi_its_mapem_ts_msgs {
    using namespace msg;
    namespace gm = geometry_msgs::msg;
}

// Implementation
#include <etsi_its_msgs_utils/impl/mapem/mapem_ts_access.h>