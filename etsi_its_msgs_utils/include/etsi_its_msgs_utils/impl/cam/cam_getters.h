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
 * @return longitudinal acceleration in m/s^2 as decimal number (accelerating is positive)
 */
inline double getLongitudinalAcceleration(const LongitudinalAcceleration& longitudinal_acceleration) {
  return ((double)longitudinal_acceleration.longitudinal_acceleration_value.value) * 1e-1;
}

/**
 * @brief Get the Longitudinal Acceleration Confidence
 * 
 * @param longitudinal_acceleration to get the LongitudinalAccelerationConfidence from
 * @return double standard deviation of the longitudinal acceleration in m/s^2 as decimal number
 */
inline double getLongitudinalAccelerationConfidence(const LongitudinalAcceleration& longitudinal_acceleration) {
  return ((double)longitudinal_acceleration.longitudinal_acceleration_confidence.value) * 1e-1 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
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

/**
 * @brief Get the Lateral Acceleration Confidence
 * 
 * @param longitudinal_acceleration to get the LateralAccelerationConfidence from
 * @return double standard deviation of the lateral acceleration in m/s^2 as decimal number
 */
inline double getLateralAccelerationConfidence(const LateralAcceleration& lateral_acceleration) {
  return ((double)lateral_acceleration.lateral_acceleration_confidence.value) * 1e-1 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
}

#include <etsi_its_msgs_utils/impl/cam/cam_getters_common.h>

/**
 * @brief Get the confidence ellipse of the reference position as Covariance matrix
 * 
 * The covariance matrix will have the entries cov_xx, cov_xy, cov_yx, cov_yy
 * where x is the longitudinal axis and y is the lateral axis of the vehicle.
 * 
 * @param cam The CAM message to get the reference position from
 * @return const std::array<double, 4> the covariance matrix, as specified above
 */
inline const std::array<double, 4> getRefPosConfidence(const CAM& cam) {
  double object_heading = getHeading(cam) * M_PI / 180.0;
  return getPosConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse, object_heading);
}

/**
 * @brief Get the confidence ellipse of the reference position as Covariance matrix
 * 
 * The covariance matrix will have the entries cov_xx, cov_xy, cov_yx, cov_yy
 * where x is WGS84 North and y is East
 * 
 * @param cam The CAM message to get the reference position from
 * @return const std::array<double, 4> the covariance matrix, as specified above
 */
inline const std::array<double, 4> getWGSRefPosConfidence(const CAM& cam) {
  return getWGSPosConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse);
}

}  // namespace etsi_its_cam_msgs::access