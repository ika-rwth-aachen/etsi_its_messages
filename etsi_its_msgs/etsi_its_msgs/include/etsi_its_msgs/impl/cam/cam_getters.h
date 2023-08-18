/**
 * @file
 * @brief Getter functions for etsi_its_cam_msgs CAM
 */

#pragma once

namespace etsi_its_cam_msgs {

namespace access {

  /**
   * @brief Get the Station ID object
   * 
   * @param cam CAM to get the StationID value from
   * @return stationID value
   */
  inline int getStationID(const CAM& cam){
    return etsi_its_msgs::cdd_access::getStationID(cam.header);
  }

  /**
   * @brief Get the stationType object
   * 
   * @param cam CAM to get the stationType value from
   * @return stationType value
   */
  inline double getStationType(const CAM& cam){
    return cam.cam.camParameters.basicContainer.stationType.value;
  }

  /**
   * @brief Get the Latitude value of CAM
   * 
   * @param cam CAM to get the Latitude value from
   * @return Latitude value in degree as decimal number
   */
  inline double getLatitude(const CAM& cam){
    return etsi_its_msgs::cdd_access::getLatitude(cam.cam.camParameters.basicContainer.referencePosition.latitude);
  }

  /**
   * @brief Get the Longitude value of CAM
   * 
   * @param cam CAM to get the Longitude value from
   * @return Longitude value in degree as decimal number
   */
  inline double getLongitude(const CAM& cam){
    return etsi_its_msgs::cdd_access::getLongitude(cam.cam.camParameters.basicContainer.referencePosition.longitude);
  }

  /**
   * @brief Get the Altitude value of CAM
   * 
   * @param cam CAM to get the Altitude value from
   * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline double getAltitude(const CAM& cam){
    return etsi_its_msgs::cdd_access::getAltitude(cam.cam.camParameters.basicContainer.referencePosition.altitude);
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
    return etsi_its_msgs::cdd_access::getHeading(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading);
  }

  /**
   * @brief Get the Vehicle Length
   * 
   * @param cam CAM to get the vehicle length value from
   * @return vehicle length value in meter as decimal number
   */
  inline double getVehicleLength(const CAM& cam){
    return etsi_its_msgs::cdd_access::getVehicleLength(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleLength);
  }

  /**
   * @brief Get the Vehicle Width
   * 
   * @param cam CAM to get the vehicle width value from
   * @return vehicle width value in meter as decimal number
   */
  inline double getVehicleWidth(const CAM& cam){
    return etsi_its_msgs::cdd_access::getVehicleWidth(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleWidth);
  }

  /**
   * @brief Get the vehicle speed
   * 
   * @param cam CAM to get the speed value from
   * @return speed value in m/s as decimal number
   */
  inline double getSpeed(const CAM& cam){
    return etsi_its_msgs::cdd_access::getSpeed(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed);
  }

  /**
   * @brief Get the lateral acceleration
   * 
   * @param cam CAM to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLongitudinalAcceleration(const CAM& cam){
    return etsi_its_msgs::cdd_access::getLongitudinalAcceleration(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.longitudinalAcceleration);
  }

  /**
   * @brief Get the lateral acceleration
   * 
   * @param cam CAM to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLateralAcceleration(const CAM& cam){
    if(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration_isPresent)
    {
      return etsi_its_msgs::cdd_access::getLateralAcceleration(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration);
    }
    else
    {
      throw std::invalid_argument("LateralAcceleration is not present!");
    }
  }

} // namespace access

} // namespace etsi_its_cam_msgs
