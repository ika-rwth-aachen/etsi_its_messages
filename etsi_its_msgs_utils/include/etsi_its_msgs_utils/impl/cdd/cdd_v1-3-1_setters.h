/*
=============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University

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
 * @file impl/cdd/cdd_v1-3-1_setters.h
 * @brief Setter functions for the ETSI ITS Common Data Dictionary (CDD) v1.3.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H

#include <etsi_its_msgs_utils/impl/cdd/cdd_setters_common.h>
#include <etsi_its_msgs_utils/impl/cdd/cdd_checks.h>
#include <GeographicLib/UTMUPS.hpp>
#include <cstring>

/**
 * @brief Set the Station Id object
 *
 * @param station_id
 * @param id_value
 */
inline void setStationId(StationID& station_id, const uint32_t id_value) {
  throwIfOutOfRange(id_value, StationID::MIN, StationID::MAX, "StationID");
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
  throwIfOutOfRange(message_id, ItsPduHeader::MESSAGE_ID_MIN, ItsPduHeader::MESSAGE_ID_MAX, "MessageID");
  header.message_id = message_id;
  throwIfOutOfRange(protocol_version, ItsPduHeader::PROTOCOL_VERSION_MIN, ItsPduHeader::PROTOCOL_VERSION_MAX,
                    "ProtocolVersion");
  header.protocol_version = protocol_version;
}

/**
 * @brief Set the Station Type
 *
 * @param station_type
 * @param value
 */
inline void setStationType(StationType& station_type, const uint8_t value) {
  throwIfOutOfRange(value, StationType::MIN, StationType::MAX, "StationType");
  station_type.value = value;
}

/**
 * @brief Set the LongitudinalAccelerationValue object
 *
 * @param accel object to set
 * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
 */
inline void setLongitudinalAccelerationValue(LongitudinalAccelerationValue& accel, const double value) {
  int64_t accel_val = (int64_t)std::round(value * 1e1);
  if (accel_val >= LongitudinalAccelerationValue::MIN && accel_val <= LongitudinalAccelerationValue::MAX) {
    accel.value = accel_val;
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
 */
inline void setLongitudinalAcceleration(LongitudinalAcceleration& accel, const double value) {
  accel.longitudinal_acceleration_confidence.value = AccelerationConfidence::UNAVAILABLE;
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
 */
inline void setLateralAcceleration(LateralAcceleration& accel, const double value) {
  accel.lateral_acceleration_confidence.value = AccelerationConfidence::UNAVAILABLE;
  setLateralAccelerationValue(accel.lateral_acceleration_value, value);
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H