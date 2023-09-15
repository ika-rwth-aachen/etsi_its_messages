/**
 * @file
 * @brief Getter functions for etsi_its_cam_msgs CAM
 */

#pragma once

namespace cdd = etsi_its_msgs::cdd_access;
namespace etsi_its_cam_msgs {

namespace access {


  /**
   * @brief Get the Station ID object
   * 
   * @param cam CAM to get the StationID value from
   * @return stationID value
   */
  inline int getStationID(const CAM& cam){
    return cdd::getStationID(cam.header);
  }

  /**
   * @brief Get the stationType object
   * 
   * @param cam CAM to get the stationType value from
   * @return stationType value
   */
  inline double getStationType(const CAM& cam){
    return cam.cam.cam_parameters.basic_container.station_type.value;
  }

  /**
   * @brief Get the Latitude value of CAM
   * 
   * @param cam CAM to get the Latitude value from
   * @return Latitude value in degree as decimal number
   */
  inline double getLatitude(const CAM& cam){
    return cdd::getLatitude(cam.cam.cam_parameters.basic_container.reference_position.latitude);
  }

  /**
   * @brief Get the Longitude value of CAM
   * 
   * @param cam CAM to get the Longitude value from
   * @return Longitude value in degree as decimal number
   */
  inline double getLongitude(const CAM& cam){
    return cdd::getLongitude(cam.cam.cam_parameters.basic_container.reference_position.longitude);
  }

  /**
   * @brief Get the Altitude value of CAM
   * 
   * @param cam CAM to get the Altitude value from
   * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline double getAltitude(const CAM& cam){
    return cdd::getAltitude(cam.cam.cam_parameters.basic_container.reference_position.altitude);
  }

  /**
   * @brief Get the Heading value of CAM
   * 
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West 
   * 
   * @param cam CAM to get the Heading value from
   * @return Heading value in degree as decimal number
   */
  inline double getHeading(const CAM& cam){
    return cdd::getHeading(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading);
  }

  /**
   * @brief Get the Vehicle Length
   * 
   * @param cam CAM to get the vehicle length value from
   * @return vehicle length value in meter as decimal number
   */
  inline double getVehicleLength(const CAM& cam){
    return cdd::getVehicleLength(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_length);
  }

  /**
   * @brief Get the Vehicle Width
   * 
   * @param cam CAM to get the vehicle width value from
   * @return vehicle width value in meter as decimal number
   */
  inline double getVehicleWidth(const CAM& cam){
    return cdd::getVehicleWidth(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_width);
  }

  /**
   * @brief Get the vehicle speed
   * 
   * @param cam CAM to get the speed value from
   * @return speed value in m/s as decimal number
   */
  inline double getSpeed(const CAM& cam){
    return cdd::getSpeed(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed);
  }

  /**
   * @brief Get the lateral acceleration
   * 
   * @param cam CAM to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLongitudinalAcceleration(const CAM& cam){
    return cdd::getLongitudinalAcceleration(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration);
  }

  /**
   * @brief Get the lateral acceleration
   * 
   * @param cam CAM to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLateralAcceleration(const CAM& cam){
    if(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration_is_present)
    {
      return cdd::getLateralAcceleration(cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration);
    }
    else
    {
      throw std::invalid_argument("LateralAcceleration is not present!");
    }
  }

  /**
   * @brief Get Exterior Lights as bool vector
   * 
   * @param cam CAM to get the ExteriorLights values from
   * @return std::vector<bool> 
   */
  inline std::vector<bool> getExteriorLights(const CAM& cam){
    if(cam.cam.cam_parameters.low_frequency_container_is_present) {
      if(cam.cam.cam_parameters.low_frequency_container.choice == etsi_its_cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY) {
        return cdd::getExteriorLights(cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights);
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
