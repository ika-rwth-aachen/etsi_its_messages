/*
=============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

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
 * @file impl/cam/cam_access.h
 * @brief Main CAM access implementation header
 */

#undef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CAM_CAM_GETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CAM_CAM_SETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CAM_CAM_UTILS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CHECKS_H

#pragma once

#include <cstring>
#include <iostream>
#include <map>

#include <GeographicLib/UTMUPS.hpp>

#include <etsi_its_msgs_utils/impl/constants.h>
#include <etsi_its_msgs_utils/impl/cam/cam_getters.h>
#include <etsi_its_msgs_utils/impl/cam/cam_setters.h>

namespace etsi_its_cam_msgs::access {
#include <etsi_its_msgs_utils/impl/cam/cam_utils.h>
}  // namespace etsi_its_cam_msgs::access