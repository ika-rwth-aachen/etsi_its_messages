// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file cpm_ts_access.hpp
 * @brief Main CPM access header to include in ROS 2 projects
 */

#pragma once

// Messages
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace etsi_its_cpm_ts_msgs {
using namespace msg;
namespace gm = geometry_msgs::msg;
}  // namespace etsi_its_cpm_ts_msgs

// Implementation
#include <etsi_its_msgs_utils/impl/cpm/cpm_ts_access.h>