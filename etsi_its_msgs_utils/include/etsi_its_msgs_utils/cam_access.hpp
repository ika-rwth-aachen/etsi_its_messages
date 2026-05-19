// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file cam_access.hpp
 * @brief Main CAM access header to include in ROS 2 projects
 */

#pragma once

// Messages
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace etsi_its_cam_msgs {
    using namespace msg;
    namespace gm = geometry_msgs::msg;
}

// Implementation
#include <etsi_its_msgs_utils/impl/cam/cam_access.h>