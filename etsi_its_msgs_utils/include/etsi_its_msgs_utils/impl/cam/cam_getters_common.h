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
 * @file impl/cam/cam_getters_common.h
 * @brief Common getter functions for the ETSI ITS CAM (EN and TS)
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CAM_CAM_GETTERS_COMMON_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CAM_CAM_GETTERS_COMMON_H

/**
 * @brief Get the Station ID object
 *
 * @param cam CAM to get the StationID value from
 * @return stationID value
 */
inline uint32_t getStationID(const CAM& cam) { return getStationID(cam.header); }

/**
 * @brief Get the GenerationDeltaTime
 *
 * @param cam CAM to get the GenerationDeltaTime from
 * @return GenerationDeltaTime the GenerationDeltaTime
 */
inline GenerationDeltaTime getGenerationDeltaTime(const CAM& cam) { return cam.cam.generation_delta_time; }

/**
 * @brief Get the GenerationDeltaTime-Value
 *
 * @param cam CAM to get the GenerationDeltaTime-Value from
 * @return uint16_t the GenerationDeltaTime-Value
 */
inline uint16_t getGenerationDeltaTimeValue(const CAM& cam) { return getGenerationDeltaTime(cam).value; }

/**
 * @brief Get the stationType object
 *
 * @param cam CAM to get the stationType value from
 * @return stationType value
 */
inline uint8_t getStationType(const CAM& cam) { return cam.cam.cam_parameters.basic_container.station_type.value; }

/**
 * @brief Get the Latitude value of CAM
 *
 * @param cam CAM to get the Latitude value from
 * @return Latitude value in degree as decimal number
 */
inline double getLatitude(const CAM& cam) {
  return getLatitude(cam.cam.cam_parameters.basic_container.reference_position.latitude);
}

/**
 * @brief Get the Longitude value of CAM
 *
 * @param cam CAM to get the Longitude value from
 * @return Longitude value in degree as decimal number
 */
inline double getLongitude(const CAM& cam) {
  return getLongitude(cam.cam.cam_parameters.basic_container.reference_position.longitude);
}

/**
 * @brief Get the Altitude value of CAM
 *
 * @param cam CAM to get the Altitude value from
 * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline double getAltitude(const CAM& cam) {
  return getAltitude(cam.cam.cam_parameters.basic_container.reference_position.altitude);
}

/**
 * @brief Get the Heading value
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading to get the Heading value from
 * @return Heading value in degree as decimal number
 */
inline double getHeading(const Heading& heading) { return ((double)heading.heading_value.value) * 1e-1; }

/**
 * @brief Get the Heading value of CAM
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param cam CAM to get the Heading value from
 * @return Heading value in degree as decimal number
 */
inline double getHeading(const CAM& cam) {
  return getHeading(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading);
}

/**
 * @brief Get the Vehicle Length
 *
 * @param vehicleLength to get the vehicle length value from
 * @return vehicle length value in meter as decimal number
 */
inline double getVehicleLength(const VehicleLength& vehicle_length) {
  return ((double)vehicle_length.vehicle_length_value.value) * 1e-1;
}

/**
 * @brief Get the Vehicle Length
 *
 * @param cam CAM to get the vehicle length value from
 * @return vehicle length value in meter as decimal number
 */
inline double getVehicleLength(const CAM& cam) {
  return getVehicleLength(
      cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_length);
}

/**
 * @brief Get the Vehicle Width
 *
 * @param cam CAM to get the vehicle width value from
 * @return vehicle width value in meter as decimal number
 */
inline double getVehicleWidth(const CAM& cam) {
  return getVehicleWidth(
      cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_width);
}

/**
 * @brief Get the vehicle speed
 *
 * @param cam CAM to get the speed value from
 * @return speed value in m/s as decimal number
 */
inline double getSpeed(const CAM& cam) {
  return getSpeed(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed);
}

/**
 * @brief Get the lateral acceleration
 *
 * @param cam CAM to get the lateral acceleration from
 * @return lateral acceleration in m/s^2 as decimal number (left is positive)
 */
inline double getLongitudinalAcceleration(const CAM& cam) {
  return getLongitudinalAcceleration(
      cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration);
}

/**
 * @brief Get the lateral acceleration
 *
 * @param cam CAM to get the lateral acceleration from
 * @return lateral acceleration in m/s^2 as decimal number (left is positive)
 */
inline double getLateralAcceleration(const CAM& cam) {
  if (cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
          .lateral_acceleration_is_present) {
    return getLateralAcceleration(
        cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration);
  } else {
    throw std::invalid_argument("LateralAcceleration is not present!");
  }
}

/**
 * @brief Get the UTM Position defined within the BasicContainer of the CAM
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] cam CAM to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
inline gm::PointStamped getUTMPosition(const CAM& cam, int& zone, bool& northp) {
  return getUTMPosition(cam.cam.cam_parameters.basic_container.reference_position, zone, northp);
}

/**
 * @brief Get the UTM Position defined within the BasicContainer of the CAM
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] cam CAM to get the UTM Position from
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
inline gm::PointStamped getUTMPosition(const CAM& cam) {
  int zone;
  bool northp;
  return getUTMPosition(cam.cam.cam_parameters.basic_container.reference_position, zone, northp);
}

/**
 * @brief Get the Exterior Lights in form of bool vector
 *
 * @param exterior_lights
 * @return std::vector<bool>
 */
inline std::vector<bool> getExteriorLights(const ExteriorLights& exterior_lights) {
  return getBitString(exterior_lights.value, exterior_lights.bits_unused);
}

/**
 * @brief Get Exterior Lights as bool vector
 *
 * @param cam CAM to get the ExteriorLights values from
 * @return std::vector<bool>
 */
inline std::vector<bool> getExteriorLights(const CAM& cam) {
  if (cam.cam.cam_parameters.low_frequency_container_is_present) {
    if (cam.cam.cam_parameters.low_frequency_container.choice ==
        LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY) {
      return getExteriorLights(
          cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights);
    } else {
      throw std::invalid_argument("LowFrequencyContainer is not BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY!");
    }
  } else {
    throw std::invalid_argument("LowFrequencyContainer is not present!");
  }
}

/**
 * @brief Get Acceleration Control in form of bool vector
 *
 * @param acceleration_control
 * @return std::vector<bool>
 */
inline std::vector<bool> getAccelerationControl(const AccelerationControl& acceleration_control) {
  return getBitString(acceleration_control.value, acceleration_control.bits_unused);
}

/**
 * @brief Get the Driving Lane Status in form of bool vector
 *
 * @param driving_lane_status
 * @return std::vector<bool>
 */
inline std::vector<bool> getDrivingLaneStatus(const DrivingLaneStatus& driving_lane_status) {
  return getBitString(driving_lane_status.value, driving_lane_status.bits_unused);
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

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CAM_CAM_GETTERS_COMMON_H