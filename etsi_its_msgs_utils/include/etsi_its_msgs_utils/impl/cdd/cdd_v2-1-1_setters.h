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
 * @file impl/cdd/cdd_v2-1-1_setters.h
 * @brief Setter functions for the ETSI ITS Common Data Dictionary (CDD) v2.1.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_SETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_SETTERS_H

#include <etsi_its_msgs_utils/impl/cdd/cdd_setters_common.h>
#include <etsi_its_msgs_utils/impl/checks.h>
#include <GeographicLib/UTMUPS.hpp>
#include <cstring>

/**
 * @brief Set the Station Id object
 *
 * @param station_id
 * @param id_value
 */
inline void setStationId(StationId& station_id, const uint32_t id_value) {
  throwIfOutOfRange(id_value, StationId::MIN, StationId::MAX, "StationId");
  station_id.value = id_value;
}

/**
 * @brief Set the Its Pdu Header object
 *
 * @param header ItsPduHeader to be set
 * @param message_id ID of the message
 * @param station_id
 * @param protocol_version
 */
inline void setItsPduHeader(ItsPduHeader& header, const uint8_t message_id, const uint32_t station_id,
                            const uint8_t protocol_version = 0) {
  setStationId(header.station_id, station_id);
  throwIfOutOfRange(message_id, MessageId::MIN, MessageId::MAX, "MessageID");
  header.message_id.value = message_id;
  throwIfOutOfRange(protocol_version, OrdinalNumber1B::MIN, OrdinalNumber1B::MAX, "ProtocolVersion");
  header.protocol_version.value = protocol_version;
}

/**
 * @brief Set the Station Type
 *
 * @param station_type
 * @param value
 */
inline void setStationType(TrafficParticipantType& station_type, const uint8_t value) {
  throwIfOutOfRange(value, TrafficParticipantType::MIN, TrafficParticipantType::MAX, "StationType");
  station_type.value = value;
}

/**
 * @brief Set the LongitudinalAccelerationValue object
 *
 * @param accel object to set
 * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
 */
inline void setLongitudinalAccelerationValue(AccelerationValue& accel, const double value) {
  int64_t accel_val = (int64_t)std::round(value * 1e1);
  if (accel_val >= AccelerationValue::MIN && accel_val <= AccelerationValue::MAX) {
    accel.value = accel_val;
  } else if (accel_val < AccelerationValue::MIN) {
    accel.value = AccelerationValue::MIN;
  } else if (accel_val > AccelerationValue::MAX) {
    accel.value = AccelerationValue::MAX - 1;
  }
}

/**
 * @brief Set the LongitudinalAcceleration object
 *
 * AccelerationConfidence is set to UNAVAILABLE
 *
 * @param accel object to set
 * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
 */
inline void setLongitudinalAcceleration(AccelerationComponent& accel, const double value, const double confidence) {
  setAccelerationConfidence(accel.confidence, confidence);
  setLongitudinalAccelerationValue(accel.value, value);
}

/**
 * @brief Set the LateralAccelerationValue object
 *
 * @param accel object to set
 * @param value LateralAccelerationValue in m/s^2 as decimal number (left is positive)
 */
inline void setLateralAccelerationValue(AccelerationValue& accel, const double value) {
  int64_t accel_val = (int64_t)std::round(value * 1e1);
  if (accel_val >= AccelerationValue::MIN && accel_val <= AccelerationValue::MAX) {
    accel.value = accel_val;
  } else if (accel_val < AccelerationValue::MIN) {
    accel.value = AccelerationValue::MIN;
  } else if (accel_val > AccelerationValue::MAX) {
    accel.value = AccelerationValue::MAX - 1;
  }
}

/**
 * @brief Set the LateralAcceleration object
 *
 * AccelerationConfidence is set to UNAVAILABLE
 *
 * @param accel object to set
 * @param value LaterallAccelerationValue in m/s^2 as decimal number (left is positive)
 */
inline void setLateralAcceleration(AccelerationComponent& accel, const double value, const double confidence) {
  setAccelerationConfidence(accel.confidence, confidence);
  setLateralAccelerationValue(accel.value, value);
}

/**
 * @brief Set the Position Confidence Ellipse object
 * 
 * @param position_confidence_ellipse The position confidence ellipse to set
 * @param semi_major_axis The length of the semi-major axis in meters
 * @param semi_minor_axis The length of the semi-minor axis in meters
 * @param orientation The orientation of the semi-major axis in degrees, relative to WGS84
 */
template <typename PositionConfidenceEllipse, typename Wgs84AngleValue = decltype(PositionConfidenceEllipse::semi_major_axis_orientation)>
inline void setPositionConfidenceEllipse(PositionConfidenceEllipse& position_confidence_ellipse, const double semi_major_axis,
  const double semi_minor_axis, const double orientation) {
  setSemiAxis(position_confidence_ellipse.semi_major_axis_length, semi_major_axis);
  setSemiAxis(position_confidence_ellipse.semi_minor_axis_length, semi_minor_axis);
  setHeadingValue(position_confidence_ellipse.semi_major_axis_orientation, orientation);
}

/**
 * @brief Set the Position Confidence Ellipse object
 * 
 * @param position_confidence_ellipse 
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is aligned with the object (x = longitudinal, y = lateral)
 * @param object_heading The heading of the object in rad, with respect to WGS84
 */
template <typename PositionConfidenceEllipse>
inline void setPositionConfidenceEllipse(PositionConfidenceEllipse& position_confidence_ellipse, const std::array<double, 4>& covariance_matrix, const double object_heading){
  auto [semi_major_axis, semi_minor_axis, orientation] = confidenceEllipseFromCovMatrix(covariance_matrix, object_heading);
  setPositionConfidenceEllipse(position_confidence_ellipse, semi_major_axis, semi_minor_axis, orientation);
}

/**
 * @brief Set the Position Confidence Ellipse object
 * 
 * @param position_confidence_ellipse 
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is aligned with the WGS axes (x = North, y = East)
 * @param object_heading The heading of the object in rad, with respect to WGS84
 */
template <typename PositionConfidenceEllipse>
inline void setWGSPositionConfidenceEllipse(PositionConfidenceEllipse& position_confidence_ellipse, const std::array<double, 4>& covariance_matrix){
  auto [semi_major_axis, semi_minor_axis, orientation] = confidenceEllipseFromWGSCovMatrix(covariance_matrix);
  setPositionConfidenceEllipse(position_confidence_ellipse, semi_major_axis, semi_minor_axis, orientation);
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_SETTERS_H