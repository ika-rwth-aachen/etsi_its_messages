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
 * @file impl/cam/cam_setters.h
 * @brief Setter functions for the ETSI ITS CAM (EN)
 */

#pragma once

namespace etsi_its_cam_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v1-3-1_setters.h>

/**
 * @brief Set the ItsPduHeader-object for a CAM
 *
 * @param cam CAM-Message to set the ItsPduHeader
 * @param station_id
 * @param protocol_version
 */
inline void setItsPduHeader(CAM &cam, const uint32_t station_id, const uint8_t protocol_version = 0) {
  setItsPduHeader(cam.header, ItsPduHeader::MESSAGE_ID_CAM, station_id, protocol_version);
}

/**
 * @brief Set the LongitudinalAccelerationValue object
 *
 * @param accel object to set
 * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
 */
inline void setLongitudinalAccelerationValue(LongitudinalAccelerationValue& accel, const double value) {
  auto accel_val = std::round(value * 1e1);
  if (accel_val >= LongitudinalAccelerationValue::MIN && accel_val <= LongitudinalAccelerationValue::MAX) {
    accel.value = static_cast<int16_t>(accel_val);
  } else if (accel_val < LongitudinalAccelerationValue::MIN) {
    accel.value = LongitudinalAccelerationValue::MIN;
  } else if (accel_val > LongitudinalAccelerationValue::MAX) {
    accel.value = LongitudinalAccelerationValue::MAX - 1;
  }
}

/**
 * @brief Set the LongitudinalAcceleration object
 *
 * AccelerationConfidence is set to UNAVAILABLE
 *
 * @param accel object to set
 * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
 * @param confidence standard deviation of the longitudinal acceleration in m/s^2 as decimal number
 *                   Default is infinity, mapping to AccelerationConfidence::UNAVAILABLE
 */
inline void setLongitudinalAcceleration(LongitudinalAcceleration& accel, const double value, const double confidence = std::numeric_limits<double>::infinity()) {
  setAccelerationConfidence(accel.longitudinal_acceleration_confidence, confidence);
  setLongitudinalAccelerationValue(accel.longitudinal_acceleration_value, value);
}

/**
 * @brief Set the LateralAccelerationValue object
 *
 * @param accel object to set
 * @param value LateralAccelerationValue in m/s^2 as decimal number (left is positive)
 */
inline void setLateralAccelerationValue(LateralAccelerationValue& accel, const double value) {
  int64_t accel_val = (int64_t)std::round(value * 1e1);
  if (accel_val >= LateralAccelerationValue::MIN && accel_val <= LateralAccelerationValue::MAX) {
    accel.value = accel_val;
  } else if (accel_val < LateralAccelerationValue::MIN) {
    accel.value = LateralAccelerationValue::MIN;
  } else if (accel_val > LateralAccelerationValue::MAX) {
    accel.value = LateralAccelerationValue::MAX - 1;
  }
}

/**
 * @brief Set the LateralAcceleration object
 *
 * AccelerationConfidence is set to UNAVAILABLE
 *
 * @param accel object to set
 * @param value LaterallAccelerationValue in m/s^2 as decimal number (left is positive)
 * @param confidence standard deviation of the lateral acceleration in m/s^2 as decimal number
 *                   Default is infinity, mapping to AccelerationConfidence::UNAVAILABLE
 */
inline void setLateralAcceleration(LateralAcceleration& accel, const double value, const double confidence = std::numeric_limits<double>::infinity()) {
  setAccelerationConfidence(accel.lateral_acceleration_confidence, confidence);
  setLateralAccelerationValue(accel.lateral_acceleration_value, value);
}

#include <etsi_its_msgs_utils/impl/cam/cam_setters_common.h>

/**
 * @brief Set the confidence of the reference position
 * 
 * @param cam CAM-Message to set the confidence
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is aligned with the object (x = longitudinal, y = lateral)
 * @param object_heading heading of the object in rad, with respect to WGS84
 */
inline void setRefPosConfidence(CAM& cam, const std::array<double, 4>& covariance_matrix, const double object_heading) {
  // First ensure, that the object has the correct heading by setting its value
  double orientation = object_heading * 180 / M_PI; // Convert to degrees
  // Normalize to [0, 360)
  orientation = std::fmod(orientation + 360, 360);
  while (orientation < 0) {
    orientation += 360;
  }
  while (orientation >= 360) {
    orientation -= 360;
  }
  setHeading(cam, orientation);
  setPosConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse,
                          covariance_matrix, object_heading);
}

/**
 * @brief Set the confidence of the reference position
 * 
 * @param cam CAM-Message to set the confidence
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is WGS84 (x = North, y = East)
 */
inline void setWGSRefPosConfidence(CAM& cam, const std::array<double, 4>& covariance_matrix) {
  setWGSPosConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse,
                             covariance_matrix);
}

}  // namespace etsi_its_cam_msgs::access
