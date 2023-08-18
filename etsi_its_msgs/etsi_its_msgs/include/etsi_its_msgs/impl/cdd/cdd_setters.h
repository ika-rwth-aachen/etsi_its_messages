/**
 * @file
 * @brief Setter functions for the ETSI ITS Common Data Dictionary (CDD)
 */

#pragma once


namespace etsi_its_cam_msgs {

namespace access_functions {

  /**
   * @brief Set the Station Id object
   * 
   * @param station_id 
   * @param id_value 
   */
  inline void setStationId(StationID& station_id, int id_value) {
    if(id_value>=StationID::MIN && id_value<=StationID::MAX) station_id.value = id_value;
    else throw std::invalid_argument("StationID value is out of Range ("+std::to_string(StationID::MIN)+"..."+std::to_string(StationID::MAX)+")!");
  }

  /**
   * @brief Set the Its Pdu Header object
   * 
   * @param header ItsPduHeader to be set 
   * @param message_id ID of the message
   * @param station_id 
   * @param protocol_version 
   */
  inline void setItsPduHeader(ItsPduHeader& header, int message_id, int station_id, int protocol_version=0) {
    setStationId(header.stationID, station_id);
    if(message_id>=ItsPduHeader::MESSAGE_I_D_MIN && message_id<=ItsPduHeader::MESSAGE_I_D_MAX) header.messageID = message_id;
    else throw std::invalid_argument("messageID is out of Range ("+std::to_string(ItsPduHeader::MESSAGE_I_D_MIN)+"..."+std::to_string(ItsPduHeader::MESSAGE_I_D_MAX)+")!");
    if(protocol_version>=ItsPduHeader::PROTOCOL_VERSION_MIN && protocol_version<=ItsPduHeader::PROTOCOL_VERSION_MAX) header.protocolVersion = protocol_version;
    else throw std::invalid_argument("ProtocolVersion is out of Range ("+std::to_string(ItsPduHeader::PROTOCOL_VERSION_MIN)+"..."+std::to_string(ItsPduHeader::PROTOCOL_VERSION_MAX)+")!");
  }

  /**
   * @brief Set the Station Type
   * 
   * @param station_type 
   * @param value 
   */
  inline void setStationType(StationType& station_type, int value) {
    if(value>=StationType::MIN && value<=StationType::MAX) station_type.value = value;
    else throw std::invalid_argument("StationType value is out of Range ("+std::to_string(StationType::MIN)+"..."+std::to_string(StationType::MAX)+")!");
  }

  /**
   * @brief Set the Latitude object
   * 
   * @param latitude object to set
   * @param deg Latitude value in degree as decimal number
   */
  inline void setLatitude(Latitude& latitude, double deg) {
    int64_t angle_in_10_micro_degree = (int64_t)(deg*1e7);
    if(angle_in_10_micro_degree>=Latitude::MIN && angle_in_10_micro_degree<=Latitude::MAX) latitude.value = angle_in_10_micro_degree;
    else throw std::invalid_argument("Latitude value is out of Range ("+std::to_string(Latitude::MIN)+"..."+std::to_string(Latitude::MAX)+")!");
  }

  /**
   * @brief Set the Longitude object
   * 
   * @param longitude object to set
   * @param deg Longitude value in degree as decimal number
   */
  inline void setLongitude(Longitude& longitude, double deg) {
    int64_t angle_in_10_micro_degree = (int64_t)std::round(deg*1e7);
    if(angle_in_10_micro_degree>=Longitude::MIN && angle_in_10_micro_degree<=Longitude::MAX) longitude.value = angle_in_10_micro_degree;
    else throw std::invalid_argument("Longitude value is out of Range ("+std::to_string(Longitude::MIN)+"..."+std::to_string(Longitude::MAX)+")!");
  }

  /**
   * @brief Set the AltitudeValue object
   * 
   * @param altitude object to set
   * @param value AltitudeValue value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setAltitudeValue(AltitudeValue& altitude, double value) {
    int64_t alt_in_cm = (int64_t)std::round(value*1e2);
    if(alt_in_cm>=AltitudeValue::MIN && alt_in_cm<=AltitudeValue::MAX) altitude.value = alt_in_cm;
    else if(alt_in_cm<AltitudeValue::MIN) altitude.value = AltitudeValue::MIN;
    else if(alt_in_cm>AltitudeValue::MAX) altitude.value = AltitudeValue::MAX;
  }

  /**
   * @brief Set the Altitude object
   *
   * AltitudeConfidence is set to UNAVAILABLE
   * 
   * @param altitude object to set
   * @param value Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setAltitude(Altitude& altitude, double value) {
    altitude.altitudeConfidence.value = AltitudeConfidence::UNAVAILABLE;
    setAltitudeValue(altitude.altitudeValue, value);
  }

  /**
   * @brief Set the Reference Position object
   * 
   * @param ref_position object to set
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   * Altitude is set to UNAVAILABLE
   */
  inline void setReferencePosition(ReferencePosition& ref_position, double latitude, double longitude)
  {
    setLatitude(ref_position.latitude, latitude);
    setLongitude(ref_position.longitude, longitude);
    setAltitude(ref_position.altitude, ((double)AltitudeValue::UNAVAILABLE)/1e2);
  }

  /**
   * @brief Set the Reference Position object
   * 
   * @param ref_position object to set
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   * @param altitude Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setReferencePosition(ReferencePosition& ref_position, double latitude, double longitude, double altitude)
  {
    setLatitude(ref_position.latitude, latitude);
    setLongitude(ref_position.longitude, longitude);
    setAltitude(ref_position.altitude, altitude);
  }

  /**
   * @brief Set the HeadingValue object
   * 
   * @param heading object to set
   * @param value Heading value in degree as decimal number
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
   */
  inline void setHeadingValue(HeadingValue& heading, double value) {
    int64_t deg = (int64_t)std::round(value*1e1);
    if(deg>=HeadingValue::MIN && deg<=HeadingValue::MAX) heading.value = deg;
    else throw std::invalid_argument("HeadingValue is out of Range ("+std::to_string(HeadingValue::MIN)+"..."+std::to_string(HeadingValue::MAX)+")!");
  }

  /**
   * @brief Set the Heading object
   * 
   * @param heading object to set
   * @param value Heading value in degree as decimal number
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
   * HeadingConfidence is set to UNAVAILABLE
   */
  inline void setHeading(Heading& heading, double value) {
    heading.headingConfidence.value = HeadingConfidence::UNAVAILABLE;
    setHeadingValue(heading.headingValue, value);
  }

  /**
   * @brief Set the VehicleLengthValue object
   * 
   * @param vehicle_length object to set
   * @param value VehicleLengthValue in meter as decimal number
   */
  inline void setVehicleLengthValue(VehicleLengthValue& vehicle_length, double value) {
    int64_t length = (int64_t)std::round(value*1e1);
    if(length>=VehicleLengthValue::MIN && length<=VehicleLengthValue::MAX) vehicle_length.value = length;
    else throw std::invalid_argument("VehicleLengthValue is out of Range ("+std::to_string(VehicleLengthValue::MIN)+"..."+std::to_string(VehicleLengthValue::MAX)+")!");
  }

  /**
   * @brief Set the VehicleLength object
   * 
   * @param vehicle_length object to set
   * @param value  VehicleLengthValue in meter as decimal number
   * VehicleLengthConfidenceIndication is set to UNAVAILABLE
   */
  inline void setVehicleLength(VehicleLength& vehicle_length, double value) {
    vehicle_length.vehicleLengthConfidenceIndication.value = VehicleLengthConfidenceIndication::UNAVAILABLE;
    setVehicleLengthValue(vehicle_length.vehicleLengthValue, value);
  }

  /**
   * @brief Set the VehicleWidth object
   * 
   * @param vehicle_width object to set
   * @param value VehicleWidth in meter as decimal number
   */
  inline void setVehicleWidth(VehicleWidth& vehicle_width, double value) {
    int64_t width = (int64_t)std::round(value*1e1);
    if(width>=VehicleWidth::MIN && width<=VehicleWidth::MAX) vehicle_width.value = width;
    else throw std::invalid_argument("VehicleLengthValue is out of Range ("+std::to_string(VehicleWidth::MIN)+"..."+std::to_string(VehicleWidth::MAX)+")!");
  }

  /**
   * @brief Set the SpeedValue object
   * 
   * @param speed object to set
   * @param value SpeedValue in m/s as decimal number
   */
  inline void setSpeedValue(SpeedValue& speed, double value) {
    int64_t speed_val = (int64_t)std::round(value*1e2);
    if(speed_val>=SpeedValue::MIN && speed_val<=SpeedValue::MAX) speed.value = speed_val;
    else throw std::invalid_argument("VehicleLengthValue is out of Range ("+std::to_string(SpeedValue::MIN)+"..."+std::to_string(SpeedValue::MAX)+")!");
  }

  /**
   * @brief Set the Speed object
   * 
   * @param speed object to set
   * @param value  Speed in in m/s as decimal number
   * SpeedConfidence is set to UNAVAILABLE
   */
  inline void setSpeed(Speed& speed, double value) {
    speed.speedConfidence.value = SpeedConfidence::UNAVAILABLE;
    setSpeedValue(speed.speedValue, value);
  }

  /**
   * @brief Set the LongitudinalAccelerationValue object
   * 
   * @param accel object to set
   * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
   */
  inline void setLongitudinalAccelerationValue(LongitudinalAccelerationValue& accel, double value) {
    int64_t accel_val = (int64_t)std::round(value*1e1);
    if(accel_val>=LongitudinalAccelerationValue::MIN && accel_val<=LongitudinalAccelerationValue::MAX) accel.value = accel_val;
    else if(accel_val<LongitudinalAccelerationValue::MIN) accel.value = LongitudinalAccelerationValue::MIN;
    else if(accel_val>LongitudinalAccelerationValue::MAX) accel.value = LongitudinalAccelerationValue::MAX-1;
  }

  /**
   * @brief Set the LongitudinalAcceleration object
   * 
   * @param accel object to set
   * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
   * AccelerationConfidence is set to UNAVAILABLE
   */
  inline void setLongitudinalAcceleration(LongitudinalAcceleration& accel, double value) {
    accel.longitudinalAccelerationConfidence.value = AccelerationConfidence::UNAVAILABLE;
    setLongitudinalAccelerationValue(accel.longitudinalAccelerationValue, value);
  }

    /**
   * @brief Set the LateralAccelerationValue object
   * 
   * @param accel object to set
   * @param value LateralAccelerationValue in m/s^2 as decimal number (left is positive)
   */
  inline void setLateralAccelerationValue(LateralAccelerationValue& accel, double value) {
    int64_t accel_val = (int64_t)std::round(value*1e1);
    if(accel_val>=LateralAccelerationValue::MIN && accel_val<=LateralAccelerationValue::MAX) accel.value = accel_val;
    else if(accel_val<LateralAccelerationValue::MIN) accel.value = LateralAccelerationValue::MIN;
    else if(accel_val>LateralAccelerationValue::MAX) accel.value = LateralAccelerationValue::MAX-1;
  }

  /**
   * @brief Set the LaterallAcceleration object
   * 
   * @param accel object to set
   * @param value LaterallAccelerationValue in m/s^2 as decimal number (left is positive)
   * AccelerationConfidence is set to UNAVAILABLE
   */
  inline void setLateralAcceleration(LateralAcceleration& accel, double value) {
    accel.lateralAccelerationConfidence.value = AccelerationConfidence::UNAVAILABLE;
    setLateralAccelerationValue(accel.lateralAccelerationValue, value);
  }

} // namespace access_functions

} // namespace etsi_its_cam_msgs
