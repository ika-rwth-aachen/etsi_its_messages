/**
 * @file
 * @brief Getter functions for the ETSI ITS Common Data Dictionary (CDD)
 */

#pragma once

namespace etsi_its_msgs {

namespace cdd_access {

  /**
  * @brief Get the StationID of ItsPduHeader
  * 
  * @param header ItsPduHeader to get the StationID value from
  * @return stationID value
  */
  inline int getStationID(const ItsPduHeader& header){
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
   * @brief Get the lateral acceleration
   * 
   * @param longitudinalAcceleration to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLongitudinalAcceleration(const LongitudinalAcceleration& longitudinal_acceleration){
    return ((double)longitudinal_acceleration.longitudinal_acceleration_value.value)*1e-1;
  }

  /**
   * @brief Get the lateral acceleration
   * 
   * @param lateralAcceleration to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLateralAcceleration(const LateralAcceleration& lateral_acceleration){
    return ((double)lateral_acceleration.lateral_acceleration_value.value)*1e-1;
  }

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

  inline std::vector<bool> getExteriorLights(const ExteriorLights& exterior_lights){
    return getBitString(exterior_lights.value, exterior_lights.bits_unused);
  }

} // namespace access

} // namespace etsi_its_msgs
