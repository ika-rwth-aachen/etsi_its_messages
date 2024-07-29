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
#pragma once

#include <GeographicLib/UTMUPS.hpp>


/**
* @brief Get the StationID of ItsPduHeader
*
* @param header ItsPduHeader to get the StationID value from
* @return stationID value
*/
inline uint32_t getStationID(const ItsPduHeader& header){
  return header.station_id.value;
}

/**
 * @brief Get the Latitude value
 *
 * @param latitude to get the Latitude value from
 * @return Latitude value in degree as decimal number
 */
inline double getLatitude(const Latitude& latitude){
  return ((double)latitude.value)*1e-7;
}

/**
 * @brief Get the Longitude value
 *
 * @param longitude to get the Longitude value from
 * @return Longitude value in degree as decimal number
 */
inline double getLongitude(const Longitude& longitude){
  return ((double)longitude.value)*1e-7;
}

/**
 * @brief Get the Altitude value
 *
 * @param altitude to get the Altitude value from
 * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline double getAltitude(const Altitude& altitude){
  return ((double)altitude.altitude_value.value)*1e-2;
}

/**
 * @brief Get the Heading value
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading to get the Heading value from
 * @return Heading value in degree as decimal number
 */
inline double getHeading(const Heading& heading){
  return ((double)heading.heading_value.value)*1e-1;
}

/**
 * @brief Get the Vehicle Length
 *
 * @param vehicleLength to get the vehicle length value from
 * @return vehicle length value in meter as decimal number
 */
inline double getVehicleLength(const VehicleLength& vehicle_length){
  return ((double)vehicle_length.vehicle_length_value.value)*1e-1;
}

/**
 * @brief Get the Vehicle Width
 *
 * @param vehicleWidth to get the vehicle width value from
 * @return vehicle width value in meter as decimal number
 */
inline double getVehicleWidth(const VehicleWidth& vehicle_width){
  return ((double)vehicle_width.value)*1e-1;
}

/**
 * @brief Get the vehicle speed
 *
 * @param speed to get the speed value from
 * @return speed value in m/s as decimal number
 */
inline double getSpeed(const Speed& speed){
  return ((double)speed.speed_value.value)*1e-2;
}

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
 * @brief Get Acceleration Control in form of bool vector
 *
 * @param acceleration_control
 * @return std::vector<bool>
 */
inline std::vector<bool> getAccelerationControl(const AccelerationControl& acceleration_control){
  return getBitString(acceleration_control.value, acceleration_control.bits_unused);
}

/**
 * @brief Get the Driving Lane Status in form of bool vector
 *
 * @param driving_lane_status
 * @return std::vector<bool>
 */
inline std::vector<bool> getDrivingLaneStatus(const DrivingLaneStatus& driving_lane_status){
  return getBitString(driving_lane_status.value, driving_lane_status.bits_unused);
}

/**
 * @brief Get the Exterior Lights in form of bool vector
 *
 * @param exterior_lights
 * @return std::vector<bool>
 */
inline std::vector<bool> getExteriorLights(const ExteriorLights& exterior_lights){
  return getBitString(exterior_lights.value, exterior_lights.bits_unused);
}

/**
 * @brief Get the Special Transport Type in form of bool vector
 *
 * @param special_transport_type
 * @return std::vector<bool>
 */
inline std::vector<bool> getSpecialTransportType(const SpecialTransportType& special_transport_type) {
  return getBitString(special_transport_type.value, special_transport_type.bits_unused);
}

/**
 * @brief Get the Lightbar Siren In Use in form of bool vector
 *
 * @param light_bar_siren_in_use
 * @return std::vector<bool>
 */
inline std::vector<bool> getLightBarSirenInUse(const LightBarSirenInUse& light_bar_siren_in_use) {
  return getBitString(light_bar_siren_in_use.value, light_bar_siren_in_use.bits_unused);
}

/**
 * @brief Get the Vehicle Role in form of bool vector
 *
 * @param vehicle_role
 * @return std::vector<bool>
 */
inline std::vector<bool> getEmergencyPriority(const EmergencyPriority& emergency_priority) {
  return getBitString(emergency_priority.value, emergency_priority.bits_unused);
}