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
 * @file impl/mcm/mcm_setters.h
 * @brief Setter functions for the UULM MCM (TS)
 */

#pragma once

#include <etsi_its_msgs_utils/impl/checks.h>

namespace etsi_its_mcm_uulm_msgs::access {

// ---------- header ----------

inline void setItsPduHeader(MCM& mcm, const uint32_t station_id, const uint8_t protocol_version = 2) {
  mcm.header.message_id.value = MessageId::MCM;
  mcm.header.station_id.value = station_id;
  throwIfOutOfRange(protocol_version, OrdinalNumber1B::MIN, OrdinalNumber1B::MAX, "ProtocolVersion");
  mcm.header.protocol_version.value = protocol_version;
}

/**
 * @brief Set the Generation Delta Time object
 *
 * @param mcm MCM to set the GenerationDeltaTime-Value for
 * @param unix_nanosecs Timestamp in unix-nanoseconds to set the GenerationDeltaTime-Value from
 * @param n_leap_seconds Number of leap seconds since 2004 for the given timestamp (Defaults to the todays number of leap seconds since 2004.)
 */
inline void setGenerationDeltaTime(MCM& mcm, const uint64_t unix_nanosecs,
                                   const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  
  uint64_t t_its = unix_nanosecs * 1e-6 + (uint64_t)(n_leap_seconds * 1e3) - etsi_its_msgs::UNIX_SECONDS_2004 * 1e3;
  uint16_t gdt_value = t_its % 65536;
  throwIfOutOfRange(gdt_value, GenerationDeltaTime::MIN, GenerationDeltaTime::MAX, "GenerationDeltaTime");
  mcm.mcm.generation_delta_time.value = gdt_value;
}


// ---------- basic_container ----------

/**
 * @brief Set the StationType for a MCM
 *
 * @param mcm MCM-Message to set the station_type value
 * @param value station_type value to set
 */
inline void setStationType(MCM& mcm, const int value) {
  throwIfOutOfRange(value, TrafficParticipantType::MIN, TrafficParticipantType::MAX, "StationType");
  mcm.mcm.mcm_parameters.basic_container.station_type.value = value;
}

/**
 * @brief Set the Latitude object
 *
 * @param latitude object to set
 * @param deg Latitude value in degree as decimal number
 */
inline void setLatitude(Latitude& latitude, const double deg) {
  int64_t angle_in_10_micro_degree = (int64_t)std::round(deg * 1e7);
  throwIfOutOfRange(angle_in_10_micro_degree, Latitude::MIN, Latitude::MAX, "Latitude");
  latitude.value = angle_in_10_micro_degree;
}

/**
 * @brief Set the Longitude object
 *
 * @param longitude object to set
 * @param deg Longitude value in degree as decimal number
 */
inline void setLongitude(Longitude& longitude, const double deg) {
  int64_t angle_in_10_micro_degree = (int64_t)std::round(deg * 1e7);
  throwIfOutOfRange(angle_in_10_micro_degree, Longitude::MIN, Longitude::MAX, "Longitude");
  longitude.value = angle_in_10_micro_degree;
}

/**
 * @brief Set the AltitudeValue object
 *
 * @param altitude object to set
 * @param value AltitudeValue value (above the reference ellipsoid surface) in meter as decimal number
 */
inline void setAltitudeValue(AltitudeValue& altitude, const double value) {
  int64_t alt_in_cm = (int64_t)std::round(value * 1e2);
  if (alt_in_cm >= AltitudeValue::MIN && alt_in_cm <= AltitudeValue::MAX) {
    altitude.value = alt_in_cm;
  } else if (alt_in_cm < AltitudeValue::MIN) {
    altitude.value = AltitudeValue::MIN;
  } else if (alt_in_cm > AltitudeValue::MAX) {
    altitude.value = AltitudeValue::MAX;
  }
}

/**
 * @brief Set the Altitude object
 *
 * AltitudeConfidence is set to UNAVAILABLE
 *
 * @param altitude object to set
 * @param value Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline void setAltitude(Altitude& altitude, const double value) {
  altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
  setAltitudeValue(altitude.altitude_value, value);
}

/**
 * @brief Set the ReferencePosition for an MCM.
 * 
 * This function sets the latitude, longitude, and altitude of the MCMs reference position.
 * If the altitude is not provided, it is set to AltitudeValue::UNAVAILABLE.
 * Resets all Confidence values to UNAVAILABLE.
 *
 * @param mcm MCM object to set the reference position in.
 * @param latitude The latitude value position in degree as decimal number.
 * @param longitude The longitude value in degree as decimal number.
 * @param altitude The altitude value (above the reference ellipsoid surface) in meter as decimal number (optional).
 */
inline void setReferencePosition(MCM& mcm, const double latitude, const double longitude, const double altitude = AltitudeValue::UNAVAILABLE) {
  setLatitude(mcm.mcm.mcm_parameters.basic_container.reference_position.latitude, latitude);
  setLongitude(mcm.mcm.mcm_parameters.basic_container.reference_position.longitude, longitude);
  if (altitude != AltitudeValue::UNAVAILABLE) {
    setAltitude(mcm.mcm.mcm_parameters.basic_container.reference_position.altitude, altitude);
  } else {
    mcm.mcm.mcm_parameters.basic_container.reference_position.altitude.altitude_value.value = AltitudeValue::UNAVAILABLE;
    mcm.mcm.mcm_parameters.basic_container.reference_position.altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
  }

  // reset PositionConfidenceEllipse to unavailable
  mcm.mcm.mcm_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_axis_length.value = SemiAxisLength::UNAVAILABLE;
  mcm.mcm.mcm_parameters.basic_container.reference_position.position_confidence_ellipse.semi_minor_axis_length.value = SemiAxisLength::UNAVAILABLE;
  mcm.mcm.mcm_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_axis_orientation.value = Wgs84AngleValue::UNAVAILABLE;
}

/**
 * @brief Set the ReferencePosition of a MCM from a given UTM-Position
 *
 * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
 * The z-Coordinate is directly used as altitude value
 * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
 *
 * @param[out] mcm MCM to set the ReferencePosition
 * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
 * @param[in] zone the UTM zone (zero means UPS) of the given position
 * @param[in] northp hemisphere (true means north, false means south)
 */
inline void setFromUTMPosition(MCM& mcm, const gm::PointStamped& utm_position, const int zone, const bool northp) {
  std::string required_frame_prefix = "utm_";
  if (utm_position.header.frame_id.rfind(required_frame_prefix, 0) != 0) {
    throw std::invalid_argument("Frame-ID of UTM Position '" + utm_position.header.frame_id +
                                "' does not start with required prefix '" + required_frame_prefix + "'!");
  }
  double latitude, longitude;
  try {
    GeographicLib::UTMUPS::Reverse(zone, northp, utm_position.point.x, utm_position.point.y, latitude, longitude);
  } catch (GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(e.what());
  }
  setReferencePosition(mcm, latitude, longitude, utm_position.point.z);
}


}  // namespace etsi_its_mcm_uulm_msgs::access
