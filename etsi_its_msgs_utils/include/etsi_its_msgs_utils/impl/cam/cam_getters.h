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
 * @file impl/cam/cam_getters.h
 * @brief Getter functions for the ETSI ITS CAM (EN)
 */

#pragma once

namespace etsi_its_cam_msgs::access {
#include <etsi_its_msgs_utils/impl/cdd/cdd_v1-3-1_getters.h>

/**
 * @brief Get the longitudinal acceleration
 *
 * @param longitudinalAcceleration to get the longitudinal acceleration from
 * @return longitudinal acceleration in m/s^2 as decimal number (left is positive)
 */
inline double getLongitudinalAcceleration(const LongitudinalAcceleration& longitudinal_acceleration) {
  return ((double)longitudinal_acceleration.longitudinal_acceleration_value.value) * 1e-1;
}

/**
 * @brief Get the lateral acceleration
 *
 * @param lateralAcceleration to get the lateral acceleration from
 * @return lateral acceleration in m/s^2 as decimal number (left is positive)
 */
inline double getLateralAcceleration(const LateralAcceleration& lateral_acceleration) {
  return ((double)lateral_acceleration.lateral_acceleration_value.value) * 1e-1;
}

#include <etsi_its_msgs_utils/impl/cam/cam_getters_common.h>

}  // namespace etsi_its_cam_msgs::access