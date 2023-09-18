/**
 * @file
 * @brief Setter functions for etsi_its_cam_msgs CAM
 */

#pragma once

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
  inline void setItsPduHeader(CAM& cam, int station_id, int protocol_version = 0){
    cdd::setItsPduHeader(cam.header, ItsPduHeader::MESSAGE_I_D_CAM, station_id, protocol_version);
  }

  /**
   * @brief Set the StationType for a CAM
   * 
   * @param cam CAM-Message to set the station_type value 
   * @param value station_type value to set
   */
  inline void setStationType(CAM& cam, const int value){
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
  inline void setReferencePosition(CAM& cam, double latitude, double longitude)
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
  inline void setReferencePosition(CAM& cam, double latitude, double longitude, double altitude)
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
