#pragma once

// Messages
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace etsi_its_cam_msgs {
    using namespace msg;
    namespace gm = geometry_msgs::msg;
}

// Implementation
#include <etsi_its_msgs/impl/cam/cam_access.h>