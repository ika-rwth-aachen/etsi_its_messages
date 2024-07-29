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
 * @file impl/cdd/cdd_v2-1-1_setters.h
 * @brief Setter functions for the ETSI ITS Common Data Dictionary (CDD) v2.1.1
 */

#pragma once

#include <cstring>
#include <etsi_its_msgs_utils/impl/cdd/cdd_setters_common.h>
#include <etsi_its_msgs_utils/impl/cdd/cdd_checks.h>
#include <etsi_its_msgs_utils/impl/constants.h>
#include <GeographicLib/UTMUPS.hpp>


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
inline void setItsPduHeader(ItsPduHeader& header, const uint8_t message_id, const uint32_t station_id, const uint8_t protocol_version=0) {
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
 * @brief Set the Reference Position object
 *
 * Altitude is set to UNAVAILABLE
 *
 * @param ref_position object to set
 * @param latitude Latitude value in degree as decimal number
 * @param longitude Longitude value in degree as decimal number
 */
inline void setReferencePosition(ReferencePositionWithConfidence& ref_position, const double latitude, const double longitude)
{
  setLatitude(ref_position.latitude, latitude);
  setLongitude(ref_position.longitude, longitude);
  ref_position.altitude.altitude_value.value  = AltitudeValue::UNAVAILABLE;
  ref_position.altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
}

/**
 * @brief Set the Reference Position object
 *
 * @param ref_position object to set
 * @param latitude Latitude value in degree as decimal number
 * @param longitude Longitude value in degree as decimal number
 * @param altitude Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline void setReferencePosition(ReferencePositionWithConfidence& ref_position, const double latitude, const double longitude, const double altitude)
{
  setLatitude(ref_position.latitude, latitude);
  setLongitude(ref_position.longitude, longitude);
  setAltitude(ref_position.altitude, altitude);
}

/**
 * @brief Set the LongitudinalAccelerationValue object
 *
 * @param accel object to set
 * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
 */
inline void setLongitudinalAccelerationValue(AccelerationValue& accel, const double value) {
  int64_t accel_val = (int64_t)std::round(value*1e1);
  if(accel_val>=AccelerationValue::MIN && accel_val<=AccelerationValue::MAX) accel.value = accel_val;
  else if(accel_val<AccelerationValue::MIN) accel.value = AccelerationValue::MIN;
  else if(accel_val>AccelerationValue::MAX) accel.value = AccelerationValue::MAX-1;
}

/**
 * @brief Set the LongitudinalAcceleration object
 *
 * AccelerationConfidence is set to UNAVAILABLE
 *
 * @param accel object to set
 * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
 */
inline void setLongitudinalAcceleration(AccelerationComponent& accel, const double value) {
  accel.confidence.value = AccelerationConfidence::UNAVAILABLE;
  setLongitudinalAccelerationValue(accel.value, value);
}

  /**
 * @brief Set the LateralAccelerationValue object
 *
 * @param accel object to set
 * @param value LateralAccelerationValue in m/s^2 as decimal number (left is positive)
 */
inline void setLateralAccelerationValue(AccelerationValue& accel, const double value) {
  int64_t accel_val = (int64_t)std::round(value*1e1);
  if(accel_val>=AccelerationValue::MIN && accel_val<=AccelerationValue::MAX) accel.value = accel_val;
  else if(accel_val<AccelerationValue::MIN) accel.value = AccelerationValue::MIN;
  else if(accel_val>AccelerationValue::MAX) accel.value = AccelerationValue::MAX-1;
}

/**
 * @brief Set the LateralAcceleration object
 *
 * AccelerationConfidence is set to UNAVAILABLE
 *
 * @param accel object to set
 * @param value LaterallAccelerationValue in m/s^2 as decimal number (left is positive)
 */
inline void setLateralAcceleration(AccelerationComponent& accel, const double value) {
  accel.confidence.value = AccelerationConfidence::UNAVAILABLE;
  setLateralAccelerationValue(accel.value, value);
}

/**
 * @brief Set the ReferencePosition from a given UTM-Position
 *
 * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
 * The z-Coordinate is directly used as altitude value
 * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
 *
 * @param[out] reference_position ReferencePosition to set
 * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
 * @param[in] zone the UTM zone (zero means UPS) of the given position
 * @param[in] northp hemisphere (true means north, false means south)
 */
inline void setFromUTMPosition(ReferencePositionWithConfidence& reference_position, const gm::PointStamped& utm_position, const int zone, const bool northp)
{
  std::string required_frame_prefix = "utm_";
  if(utm_position.header.frame_id.rfind(required_frame_prefix, 0) != 0)
  {
    throw std::invalid_argument("Frame-ID of UTM Position '"+utm_position.header.frame_id+"' does not start with required prefix '"+required_frame_prefix+"'!");
  }
  double latitude, longitude;
  try {
    GeographicLib::UTMUPS::Reverse(zone, northp, utm_position.point.x, utm_position.point.y, latitude, longitude);
  } catch (GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(e.what());
  }
  setReferencePosition(reference_position, latitude, longitude, utm_position.point.z);
}