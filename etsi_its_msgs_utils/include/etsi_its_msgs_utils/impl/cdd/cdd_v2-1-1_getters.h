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
 * @file impl/cdd/cdd_v2-1-1_getters.h
 * @brief Getter functions for the ETSI ITS Common Data Dictionary (CDD) v2.1.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_GETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_GETTERS_H

#include <GeographicLib/UTMUPS.hpp>

#include <etsi_its_msgs_utils/impl/cdd/cdd_getters_common.h>

/**
 * @brief Get the longitudinal acceleration
 *
 * @param longitudinalAcceleration to get the longitudinal acceleration from
 * @return longitudinal acceleration in m/s^2 as decimal number (left is positive)
 */
inline double getLongitudinalAcceleration(const AccelerationComponent& longitudinal_acceleration) {
  return ((int16_t)longitudinal_acceleration.value.value) * 1e-1;
}

/**
 * @brief Get the lateral acceleration
 *
 * @param lateralAcceleration to get the lateral acceleration from
 * @return lateral acceleration in m/s^2 as decimal number (left is positive)
 */
inline double getLateralAcceleration(const AccelerationComponent& lateral_acceleration) {
  return ((int16_t)lateral_acceleration.value.value) * 1e-1;
}

template <typename PositionConfidenceEllipse>
inline std::tuple<double, double, double> getPositionConfidenceEllipse(PositionConfidenceEllipse& position_confidence_ellipse) {
  return {
    getSemiAxis(position_confidence_ellipse.semi_major_axis_length),
    getSemiAxis(position_confidence_ellipse.semi_minor_axis_length),
    position_confidence_ellipse.semi_major_axis_orientation.value * 1e-1
  };
}


template <typename PositionConfidenceEllipse>
inline std::array<double, 4> getPositionConfidenceEllipse(const PositionConfidenceEllipse& position_confidence_ellipse, const double object_heading){
  auto [semi_major, semi_minor, major_orientation] = getPositionConfidenceEllipse(position_confidence_ellipse);
  return CovMatrixFromConfidenceEllipse(semi_major, semi_minor, major_orientation, object_heading);
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_GETTERS_H