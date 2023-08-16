/**
 * @file
 * @brief Getter functions for the ETSI ITS Common Data Dictionary (CDD)
 */

#pragma once


namespace etsi_its_cam_msgs {

namespace access_functions {

  /**
  * @brief Get the StationID of ItsPduHeader
  * 
  * @param header ItsPduHeader to get the StationID value from
  * @return stationID value
  */
  inline int getStationID(const ItsPduHeader& header){
    return header.stationID.value;
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
    return ((double)altitude.altitudeValue.value)*1e-2;
  }

  /**
   * @brief Get the Heading value
   * 
   * @param heading to get the Heading value from
   * @return Heading value in degree as decimal number
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West 
   */
  inline double getHeading(const Heading& heading){
    return ((double)heading.headingValue.value)*1e-1;
  }

  /**
   * @brief Get the Vehicle Length
   * 
   * @param vehicleLength to get the vehicle length value from
   * @return vehicle length value in meter as decimal number
   */
  inline double getVehicleLength(const VehicleLength& vehicleLength){
    return ((double)vehicleLength.vehicleLengthValue.value)*1e-1;
  }

  /**
   * @brief Get the Vehicle Width
   * 
   * @param vehicleWidth to get the vehicle width value from
   * @return vehicle width value in meter as decimal number
   */
  inline double getVehicleWidth(const VehicleWidth& vehicleWidth){
    return ((double)vehicleWidth.value)*1e-1;
  }

  /**
   * @brief Get the vehicle speed
   * 
   * @param speed to get the speed value from
   * @return speed value in m/s as decimal number
   */
  inline double getSpeed(const Speed& speed){
    return ((double)speed.speedValue.value)*1e-2;
  }

  /**
   * @brief Get the lateral acceleration
   * 
   * @param longitudinalAcceleration to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLongitudinalAcceleration(const LongitudinalAcceleration& longitudinalAcceleration){
    return ((double)longitudinalAcceleration.longitudinalAccelerationValue.value)*1e-1;
  }

  /**
   * @brief Get the lateral acceleration
   * 
   * @param lateralAcceleration to get the lateral acceleration from
   * @return lateral acceleration in m/s^2 as decimal number (left is positive)
   */
  inline double getLateralAcceleration(const LateralAcceleration& lateralAcceleration){
    return ((double)lateralAcceleration.lateralAccelerationValue.value)*1e-1;
  }

} // namespace access_functions

} // namespace etsi_its_cam_msgs
