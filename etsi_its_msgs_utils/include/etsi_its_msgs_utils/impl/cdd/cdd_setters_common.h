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
 * @file impl/cdd/cdd_setters_common.h
 * @brief Common setter functions for the ETSI ITS Common Data Dictionary (CDD) v1.3.1 and v2.1.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H

#include <etsi_its_msgs_utils/impl/cdd/cdd_checks.h>
#include <etsi_its_msgs_utils/impl/constants.h>
#include <GeographicLib/UTMUPS.hpp>
#include <cstring>

/**
 * @brief Set the TimestampITS object
 *
 * @param[in] timestamp_its TimestampITS object to set the timestamp
 * @param[in] unix_nanosecs Unix-Nanoseconds to set the timestamp for
 * @param[in] n_leap_seconds Number of leap-seconds since 2004. (Default: etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second)
 * @param[in] epoch_offset Unix-Timestamp in seconds for the 01.01.2004 at 00:00:00
 */
inline void setTimestampITS(
    TimestampIts& timestamp_its, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second) {
  uint64_t t_its = unix_nanosecs * 1e-6 + (uint64_t)(n_leap_seconds * 1e3) - etsi_its_msgs::UNIX_SECONDS_2004 * 1e3;
  throwIfOutOfRange(t_its, TimestampIts::MIN, TimestampIts::MAX, "TimestampIts");
  timestamp_its.value = t_its;
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
 * @brief Set the SpeedValue object
 *
 * @param speed object to set
 * @param value SpeedValue in m/s as decimal number
 */
inline void setSpeedValue(SpeedValue& speed, const double value) {
  int64_t speed_val = (int64_t)std::round(value * 1e2);
  throwIfOutOfRange(speed_val, SpeedValue::MIN, SpeedValue::MAX, "SpeedValue");
  speed.value = speed_val;
}

/**
 * @brief Set the Speed object
 *
 * SpeedConfidence is set to UNAVAILABLE
 *
 * @param speed object to set
 * @param value  Speed in in m/s as decimal number
 */
inline void setSpeed(Speed& speed, const double value) {
  speed.speed_confidence.value = SpeedConfidence::UNAVAILABLE;
  setSpeedValue(speed.speed_value, value);
}

/**
 * @brief Sets the reference position in the given ReferencePostion object.
 * 
 * This function sets the latitude, longitude, and altitude of the reference position.
 * If the altitude is not provided, it is set to AltitudeValue::UNAVAILABLE.
 * 
 * @param ref_position ReferencePostion or ReferencePositionWithConfidence object to set the reference position in.
 * @param latitude The latitude value position in degree as decimal number.
 * @param longitude The longitude value in degree as decimal number.
 * @param altitude The altitude value (above the reference ellipsoid surface) in meter as decimal number (optional).
 */
template <typename T>
inline void setReferencePosition(T& ref_position, const double latitude, const double longitude,
                                 const double altitude = AltitudeValue::UNAVAILABLE) {
  setLatitude(ref_position.latitude, latitude);
  setLongitude(ref_position.longitude, longitude);
  if (altitude != AltitudeValue::UNAVAILABLE) {
    setAltitude(ref_position.altitude, altitude);
  } else {
    ref_position.altitude.altitude_value.value = AltitudeValue::UNAVAILABLE;
    ref_position.altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
  }
  // TODO: set confidence values
}

/**
 * @brief Set the ReferencePosition from a given UTM-Position
 *
 * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
 * The z-Coordinate is directly used as altitude value
 * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
 *
 * @param[out] reference_position ReferencePostion or ReferencePositionWithConfidence to set
 * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
 * @param[in] zone the UTM zone (zero means UPS) of the given position
 * @param[in] northp hemisphere (true means north, false means south)
 */
template <typename T>
inline void setFromUTMPosition(T& reference_position, const gm::PointStamped& utm_position, const int zone,
                               const bool northp) {
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
  setReferencePosition(reference_position, latitude, longitude, utm_position.point.z);
}

/**
 * @brief Set a Bit String by a vector of bools
 *
 * @tparam T
 * @param bitstring BitString to set
 * @param bits vector of bools
 */
template <typename T>
inline void setBitString(T& bitstring, const std::vector<bool>& bits) {
  // bit string size
  const int bits_per_byte = 8;
  const int n_bytes = (bits.size() - 1) / bits_per_byte + 1;
  const int n_bits = n_bytes * bits_per_byte;

  // init output
  bitstring.bits_unused = n_bits - bits.size();
  bitstring.value = std::vector<uint8_t>(n_bytes);

  // loop over all bytes in reverse order
  for (int byte_idx = n_bytes - 1; byte_idx >= 0; byte_idx--) {
    // loop over bits in a byte
    for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {
      // map bit index in byte to bit index in total bitstring
      int bit_idx = (n_bytes - byte_idx - 1) * bits_per_byte + bit_idx_in_byte;
      if (byte_idx == 0 && bit_idx >= n_bits - bitstring.bits_unused) break;

      // set bit in output bitstring appropriately
      bitstring.value[byte_idx] |= bits[bit_idx] << bit_idx_in_byte;
    }
  }
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H