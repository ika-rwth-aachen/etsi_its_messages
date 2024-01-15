/*
=============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=============================================================================
*/

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
<<<<<<< HEAD:etsi_its_msgs_utils/include/etsi_its_msgs_utils/denm_access.hpp
#include <etsi_its_msgs/impl/denm/denm_access.h>
=======
#include <etsi_its_msgs_utils/impl/cam/cam_access.h>
>>>>>>> origin/main:etsi_its_msgs_utils/include/etsi_its_msgs_utils/cam_access.hpp
