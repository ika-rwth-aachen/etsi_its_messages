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
 * @file impl/cam/cam_setters.h
 * @brief Setter functions for the ETSI ITS CAM
 */

#pragma once

#include <etsi_its_msgs_utils/impl/constants.h>

namespace cdd = etsi_its_msgs::cdd_access;
namespace etsi_its_cam_msgs {

namespace access {

  /**
   * @brief Set the ItsPduHeader-object for a CAM
   *
   * @param cam CAM-Message to set the ItsPduHeader
   * @param station_id
   * @param protocol_version
   */
  inline void setItsPduHeader(CAM& cam, const uint32_t station_id, const uint8_t protocol_version = 0){
    cdd::setItsPduHeader(cam.header, ItsPduHeader::MESSAGE_ID_CAM, station_id, protocol_version);
  }

  /**
   * @brief Set the GenerationDeltaTime-Value
   *
   * @param generation_delta_time GenerationDeltaTime to set the GenerationDeltaTime-Value for
   * @param unix_nanosecs Timestamp in unix-nanoseconds to set the GenerationDeltaTime-Value from
   * @param n_leap_seconds Number of leap seconds since 2004 for the given timestamp (Default: etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second)
   */
  inline void setGenerationDeltaTime(GenerationDeltaTime& generation_delta_time, const uint64_t unix_nanosecs, const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second) {
    TimestampIts t_its;
    cdd::setTimestampITS(t_its, unix_nanosecs, n_leap_seconds);
    uint16_t gdt_value = t_its.value%65536;
    cdd::throwIfOutOfRange(gdt_value, GenerationDeltaTime::MIN, GenerationDeltaTime::MAX, "GenerationDeltaTime");
    generation_delta_time.value=gdt_value;
  }

  /**
   * @brief Set the Generation Delta Time object
   *
   * @param cam CAM to set the GenerationDeltaTime-Value for
   * @param unix_nanosecs Timestamp in unix-nanoseconds to set the GenerationDeltaTime-Value from
   * @param n_leap_seconds Number of leap seconds since 2004 for the given timestamp  (Default: etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second)
   */
  inline void setGenerationDeltaTime(CAM& cam, const uint64_t unix_nanosecs, const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second) {
    setGenerationDeltaTime(cam.cam.generation_delta_time, unix_nanosecs, n_leap_seconds);
  }

  /**
   * @brief Set the StationType for a CAM
   *
   * @param cam CAM-Message to set the station_type value
   * @param value station_type value to set
   */
  inline void setStationType(CAM& cam, const uint8_t value){
    cdd::setStationType(cam.cam.cam_parameters.basic_container.station_type, value);
  }

  /**
   * @brief Set the ReferencePosition for a CAM
   *
   * Altitude is set to UNAVAILABLE
   *
   * @param cam CAM to set the ReferencePosition
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   */
  inline void setReferencePosition(CAM& cam, const double latitude, const double longitude)
  {
    cdd::setReferencePosition(cam.cam.cam_parameters.basic_container.reference_position, latitude, longitude);
  }

  /**
   * @brief Set the ReferencePosition for a CAM
   *
   * @param cam CAM to set the ReferencePosition
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   * @param altitude Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setReferencePosition(CAM& cam, const double latitude, const double longitude, const double altitude)
  {
    cdd::setReferencePosition(cam.cam.cam_parameters.basic_container.reference_position, latitude, longitude, altitude);
  }

  /**
   * @brief Set the Heading for a CAM
   *
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
   * HeadingConfidence is set to UNAVAILABLE
   *
   * @param cam CAM to set the ReferencePosition
   * @param value Heading value in degree as decimal number
   */
  inline void setHeading(CAM& cam, const double heading_val){
    cdd::setHeading(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading, heading_val);
  }

  /**
   * @brief Set the vehicle dimensions
   *
   * @param cam CAM to set the vehicle dimensions
   * @param vehicle_length vehicle length in meter as decimal number
   * @param vehicle_width vehicle width in meter as decimal number
   */
  inline void setVehicleDimensions(CAM& cam, const double vehicle_length, const double vehicle_width){
    cdd::setVehicleLength(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_length, vehicle_length);
    cdd::setVehicleWidth(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_width, vehicle_width);
  }

  /**
   * @brief Set the vehicle speed
   *
   * @param cam CAM to set the speed value
   * @param speed_val speed value to set in m/s as decimal number
   */
  inline void setSpeed(CAM& cam, const double speed_val){
    cdd::setSpeed(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed, speed_val);
  }

  /**
   * @brief Set the longitudinal acceleration
   *
   * @param cam CAM to set the acceleration value s
   * @param lon_accel longitudinal acceleration to set in m/s^2 as decimal number (braking is negative), if not available use 16.1 m/s^2
   */
  inline void setLongitudinalAcceleration(CAM& cam, const double lon_accel){
    cdd::setLongitudinalAcceleration(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration, lon_accel);
  }

  /**
   * @brief Set the lateral acceleration
   *
   * @param cam CAM to set the acceleration value s
   * @param lat_accel lateral acceleration to set in m/s^2 as decimal number (left is positiv), if not available use 16.1 m/s^2
   */
  inline void setLateralAcceleration(CAM& cam, const double lat_accel){
      cdd::setLateralAcceleration(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration, lat_accel);
      cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration_is_present = true;
  }

  /**
   * @brief Set the ReferencePosition of a CAM from a given UTM-Position
   *
   * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
   * The z-Coordinate is directly used as altitude value
   * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
   *
   * @param[out] cam CAM for which to set the ReferencePosition
   * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
   * @param[in] zone the UTM zone (zero means UPS) of the given position
   * @param[in] northp hemisphere (true means north, false means south)
   */
  inline void setFromUTMPosition(CAM& cam, const gm::PointStamped& utm_position, const int& zone, const bool& northp)
  {
    cdd::setFromUTMPosition(cam.cam.cam_parameters.basic_container.reference_position, utm_position, zone, northp);
  }

  /**
   * @brief Set the Exterior Lights by using a vector of bools
   *
   * @param cam CAM to set the exterior lights
   * @param exterior_lights vector of bools to set the exterior lights
   */
  inline void setExteriorLights(CAM& cam, const std::vector<bool>& exterior_lights){
    if(etsi_its_cam_msgs::ExteriorLights::SIZE_BITS != exterior_lights.size()) {
      throw std::invalid_argument("Vector has wrong size. (" + std::to_string(exterior_lights.size()) + " != " + std::to_string(etsi_its_cam_msgs::ExteriorLights::SIZE_BITS) + ")");
    }
    if(cam.cam.cam_parameters.low_frequency_container_is_present) {
      if(cam.cam.cam_parameters.low_frequency_container.choice == etsi_its_cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY) {
        cdd::setExteriorLights(cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights, exterior_lights);
      }
      else {
        throw std::invalid_argument("LowFrequencyContainer is not BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY!");
      }
    }
    else {
      throw std::invalid_argument("LowFrequencyContainer is not present!");
    }
  }

} // namespace access

} // namespace etsi_its_cam_msgs
