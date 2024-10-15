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
 * @file impl/cdd/cdd_getters_common.h
 * @brief Common getter functions for the ETSI ITS Common Data Dictionary (CDD) v1.3.1 and v2.1.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H

#include <GeographicLib/UTMUPS.hpp>

/**
* @brief Get the StationID of ItsPduHeader
*
* @param header ItsPduHeader to get the StationID value from
* @return stationID value
*/
inline uint32_t getStationID(const ItsPduHeader& header) { return header.station_id.value; }

/**
 * @brief Get the Latitude value
 *
 * @param latitude to get the Latitude value from
 * @return Latitude value in degree as decimal number
 */
inline double getLatitude(const Latitude& latitude) { return ((double)latitude.value) * 1e-7; }

/**
 * @brief Get the Longitude value
 *
 * @param longitude to get the Longitude value from
 * @return Longitude value in degree as decimal number
 */
inline double getLongitude(const Longitude& longitude) { return ((double)longitude.value) * 1e-7; }

/**
 * @brief Get the Altitude value
 *
 * @param altitude to get the Altitude value from
 * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline double getAltitude(const Altitude& altitude) { return ((double)altitude.altitude_value.value) * 1e-2; }

/**
 * @brief Get the Vehicle Width
 *
 * @param vehicleWidth to get the vehicle width value from
 * @return vehicle width value in meter as decimal number
 */
inline double getVehicleWidth(const VehicleWidth& vehicle_width) { return ((double)vehicle_width.value) * 1e-1; }

/**
 * @brief Get the vehicle speed
 *
 * @param speed to get the speed value from
 * @return speed value in m/s as decimal number
 */
inline double getSpeed(const Speed& speed) { return ((double)speed.speed_value.value) * 1e-2; }

/**
 * @brief Get a Bit String in form of bool vector
 *
 * @param buffer as uint8_t vector
 * @param bits_unused number of bits to ignore at the end of the bit string
 * @return std::vector<bool>
 */
inline std::vector<bool> getBitString(const std::vector<uint8_t>& buffer, const int bits_unused) {
  // bit string size
  const int bits_per_byte = 8;
  const int n_bytes = buffer.size();
  const int n_bits = n_bytes * bits_per_byte;
  std::vector<bool> bits;
  bits.resize(n_bits - bits_unused, 0);

  // loop over bytes in reverse order
  for (int byte_idx = n_bytes - 1; byte_idx >= 0; byte_idx--) {
    // loop over bits in a byte
    for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {
      // map bit index in byte to bit index in total bitstring
      int bit_idx = (n_bytes - byte_idx - 1) * bits_per_byte + bit_idx_in_byte;
      if (byte_idx == 0 && bit_idx >= n_bits - bits_unused) break;

      // extract bit from bitstring and set output array entry appropriately
      bool byte_has_true_bit = buffer[byte_idx] & (1 << bit_idx_in_byte);
      if (byte_has_true_bit) bits[bit_idx] = 1;
    }
  }
  return bits;
}

/**
 * @brief Get the UTM Position defined by the given ReferencePosition
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] reference_position ReferencePosition or ReferencePositionWithConfidence to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
template <typename T>
inline gm::PointStamped getUTMPosition(const T& reference_position, int& zone, bool& northp) {
  gm::PointStamped utm_point;
  double latitude = getLatitude(reference_position.latitude);
  double longitude = getLongitude(reference_position.longitude);
  utm_point.point.z = getAltitude(reference_position.altitude);
  try {
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.point.x, utm_point.point.y);
    std::string hemisphere;
    if (northp) {
      hemisphere = "N";
    } else {
      hemisphere = "S";
    }
    utm_point.header.frame_id = "utm_" + std::to_string(zone) + hemisphere;
  } catch (GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(e.what());
  }
  return utm_point;
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H