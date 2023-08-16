/**
 * @file
 * @brief Setter functions for etsi_its_cam_msgs CAM
 */

#pragma once


namespace etsi_its_cam_msgs {

namespace access_functions {

  /**
   * @brief Set the ItsPduHeader-object for a CAM
   * 
   * @param cam CAM-Message to set the ItsPduHeader
   * @param station_id
   * @param protocol_version 
   */
  inline void setItsPduHeader(CAM& cam, int station_id, int protocol_version = 0){
    setItsPduHeader(cam.header, ItsPduHeader::MESSAGE_I_D_CAM, station_id, protocol_version);
  }

  /**
   * @brief Set the StationType for a CAM
   * 
   * @param cam CAM-Message to set the StationType value 
   * @param value StationType value to set
   */
  inline void setStationType(CAM& cam, const int value){
    setStationType(cam.cam.camParameters.basicContainer.stationType, value); 
  }

  /**
   * @brief Set the ReferencePosition for a CAM
   * 
   * @param cam CAM to set the ReferencePosition 
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   * Altitude is set to UNAVAILABLE
   */
  inline void setReferencePosition(CAM& cam, double latitude, double longitude)
  {
    setReferencePosition(cam.cam.camParameters.basicContainer.referencePosition, latitude, longitude);
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
    setReferencePosition(cam.cam.camParameters.basicContainer.referencePosition, latitude, longitude, altitude);
  }

  /**
   * @brief Set the Heading for a CAM
   * 
   * @param cam CAM- to set the ReferencePosition
   * @param value Heading value in degree as decimal number
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
   * HeadingConfidence is set to UNAVAILABLE
   */
  inline void setHeading(CAM& cam, const double heading_val){
    setHeading(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading, heading_val);
  }

  /**
   * @brief Set the vehicle dimensions
   * 
   * @param cam CAM to set the vehicle dimensions
   * @param vehicle_length vehicle length in meter as decimal number
   * @param vehicle_width vehicle width in meter as decimal number
   */
  inline void setVehicleDimensions(CAM& cam, const double vehicle_length, const double vehicle_width){
    setVehicleLength(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleLength, vehicle_length);
    setVehicleWidth(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleWidth, vehicle_width);
  }

  /**
   * @brief Set the vehicle speed
   * 
   * @param cam CAM to set the speed value 
   * @param speed_val speed value to set in m/s as decimal number
   */
  inline void setSpeed(CAM& cam, const double speed_val){
    setSpeed(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed, speed_val);
  }

  /**
   * @brief Set the lateral and longitudinal accelerations
   * 
   * @param cam CAM to set the acceleration value s
   * @param lon_accel longitudinal acceleration to set in m/s^2 as decimal number (braking is negative), if not available use 16.1 m/s^2
   * @param lat_accel lateral acceleration to set in m/s^2 as decimal number (left is positiv), if not available use 16.1 m/s^2
   */
  inline void setAccelerations(CAM& cam, const double lon_accel, const double lat_accel){
    setLongitudinalAcceleration(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.longitudinalAcceleration, lon_accel);
    if(lat_accel!=16.1)
    {
      setLateralAcceleration(cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration, lat_accel);  
      cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration_isPresent = true;
    }
    else
    {
      cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration_isPresent = false;
    }
  }
  
  // /**
  //  * @brief Set the LongitudinalAcceleration object
  //  * 
  //  * @param cam CAM-Message to set the LongitudinalAcceleration value
  //  * @param longitudinalAcceleration_val LongitudinalAcceleration value in meter/second_squared
  //  * 
  //  * set positiv = forward , negativ = backward
  //  * 
  //  */
  // inline void setLongitudinalAcceleration(CAM& cam, const double longitudinalAcceleration_val){
  //     cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue.value = longitudinalAcceleration_val; 
  // }

  // /**
  //  * @brief Set the SteeringWheelAngle object
  //  * 
  //  * @param cam CAM-Message to set the SteeringWheelAngle value
  //  * @param steeringWheelAngle_val SteeringWheelAngle value in degree
  //  * 
  //  * set positiv = left , negativ = right
  //  * 
  //  */
  // inline void setSteeringWheelAngle(CAM& cam, const double steeringWheelAngle_val){
  //     cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAngle.steeringWheelAngleValue.value = steeringWheelAngle_val/(double)1.5; 
  //     cam.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAngle_isPresent = true ;
  // }


} // namespace access_functions

} // namespace etsi_its_cam_msgs
