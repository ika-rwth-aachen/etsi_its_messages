/**
 * @file
 * @brief Setter functions for the ETSI ITS Common Data Dictionary (CDD)
 */

#include <etsi_its_msgs/impl/cdd/cdd_checks.h>

#pragma once


namespace etsi_its_msgs {

namespace cdd_access {

  /**
   * @brief Set the Station Id object
   * 
   * @param station_id 
   * @param id_value 
   */
  inline void setStationId(StationID& station_id, int id_value) {
    throwIfOutOfRange(id_value, StationID::MIN, StationID::MAX, "StationID");
    station_id.value = id_value;
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
    setStationId(header.station_i_d, station_id);
    throwIfOutOfRange(message_id, ItsPduHeader::MESSAGE_I_D_MIN, ItsPduHeader::MESSAGE_I_D_MAX, "MessageID");
    header.message_i_d = message_id;
    throwIfOutOfRange(protocol_version, ItsPduHeader::PROTOCOL_VERSION_MIN, ItsPduHeader::PROTOCOL_VERSION_MAX, "ProtocolVersion");
    header.protocol_version = protocol_version;
  }

  /**
   * @brief Set the Station Type
   * 
   * @param station_type 
   * @param value 
   */
  inline void setStationType(StationType& station_type, int value) {
    throwIfOutOfRange(value, StationType::MIN, StationType::MAX, "StationType");
    station_type.value = value;
  }

  /**
   * @brief Set the Latitude object
   * 
   * @param latitude object to set
   * @param deg Latitude value in degree as decimal number
   */
  inline void setLatitude(Latitude& latitude, double deg) {
    int64_t angle_in_10_micro_degree = (int64_t)std::round(deg*1e7);
    throwIfOutOfRange(angle_in_10_micro_degree, Latitude::MIN, Latitude::MAX, "Latitude");
    latitude.value = angle_in_10_micro_degree;
  }

  /**
   * @brief Set the Longitude object
   * 
   * @param longitude object to set
   * @param deg Longitude value in degree as decimal number
   */
  inline void setLongitude(Longitude& longitude, double deg) {
    int64_t angle_in_10_micro_degree = (int64_t)std::round(deg*1e7);
    throwIfOutOfRange(angle_in_10_micro_degree, Longitude::MIN, Longitude::MAX, "Longitude");
    longitude.value = angle_in_10_micro_degree;
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
    altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
    setAltitudeValue(altitude.altitude_value, value);
  }

  /**
   * @brief Set the Reference Position object
   * 
   * Altitude is set to UNAVAILABLE
   * 
   * @param ref_position object to set
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   */
  inline void setReferencePosition(ReferencePosition& ref_position, double latitude, double longitude)
  {
    setLatitude(ref_position.latitude, latitude);
    setLongitude(ref_position.longitude, longitude);
    ref_position.altitude.altitude_value.value  = AltitudeValue::UNAVAILABLE;
    ref_position.altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
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
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
   * 
   * @param heading object to set
   * @param value Heading value in degree as decimal number
   */
  inline void setHeadingValue(HeadingValue& heading, double value) {
    int64_t deg = (int64_t)std::round(value*1e1);
    throwIfOutOfRange(deg, HeadingValue::MIN, HeadingValue::MAX, "HeadingValue");
    heading.value = deg;
  }

  /**
   * @brief Set the Heading object
   * 
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
   * HeadingConfidence is set to UNAVAILABLE
   * 
   * @param heading object to set
   * @param value Heading value in degree as decimal number
   */
  inline void setHeading(Heading& heading, double value) {
    heading.heading_confidence.value = HeadingConfidence::UNAVAILABLE;
    setHeadingValue(heading.heading_value, value);
  }

  /**
   * @brief Set the VehicleLengthValue object
   * 
   * @param vehicle_length object to set
   * @param value VehicleLengthValue in meter as decimal number
   */
  inline void setVehicleLengthValue(VehicleLengthValue& vehicle_length, double value) {
    int64_t length = (int64_t)std::round(value*1e1);
    throwIfOutOfRange(length, VehicleLengthValue::MIN, VehicleLengthValue::MAX, "VehicleLengthValue");
    vehicle_length.value = length;
  }

  /**
   * @brief Set the VehicleLength object
   * 
   * VehicleLengthConfidenceIndication is set to UNAVAILABLE
   * 
   * @param vehicle_length object to set
   * @param value  VehicleLengthValue in meter as decimal number
   */
  inline void setVehicleLength(VehicleLength& vehicle_length, double value) {
    vehicle_length.vehicle_length_confidence_indication.value = VehicleLengthConfidenceIndication::UNAVAILABLE;
    setVehicleLengthValue(vehicle_length.vehicle_length_value, value);
  }

  /**
   * @brief Set the VehicleWidth object
   * 
   * @param vehicle_width object to set
   * @param value VehicleWidth in meter as decimal number
   */
  inline void setVehicleWidth(VehicleWidth& vehicle_width, double value) {
    int64_t width = (int64_t)std::round(value*1e1);
    throwIfOutOfRange(width, VehicleWidth::MIN, VehicleWidth::MAX, "VehicleWidthValue");
    vehicle_width.value = width;
  }

  /**
   * @brief Set the SpeedValue object
   * 
   * @param speed object to set
   * @param value SpeedValue in m/s as decimal number
   */
  inline void setSpeedValue(SpeedValue& speed, double value) {
    int64_t speed_val = (int64_t)std::round(value*1e2);
    throwIfOutOfRange(speed_val, SpeedValue::MIN, SpeedValue::MAX, "SpeedValue");
    speed.value = speed_val;
  }

  /**
   * @brief Set the Speed object
   * 
   * SpeedConfidence is set to UNAVAILABLE
   * 
   * @param speed object to set
   * @param value  Speed in in m/s as decimal number
   */
  inline void setSpeed(Speed& speed, double value) {
    speed.speed_confidence.value = SpeedConfidence::UNAVAILABLE;
    setSpeedValue(speed.speed_value, value);
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
   * AccelerationConfidence is set to UNAVAILABLE
   * 
   * @param accel object to set
   * @param value LongitudinalAccelerationValue in m/s^2 as decimal number (braking is negative)
   */
  inline void setLongitudinalAcceleration(LongitudinalAcceleration& accel, double value) {
    accel.longitudinal_acceleration_confidence.value = AccelerationConfidence::UNAVAILABLE;
    setLongitudinalAccelerationValue(accel.longitudinal_acceleration_value, value);
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
   * AccelerationConfidence is set to UNAVAILABLE
   * 
   * @param accel object to set
   * @param value LaterallAccelerationValue in m/s^2 as decimal number (left is positive)
   */
  inline void setLateralAcceleration(LateralAcceleration& accel, double value) {
    accel.lateral_acceleration_confidence.value = AccelerationConfidence::UNAVAILABLE;
    setLateralAccelerationValue(accel.lateral_acceleration_value, value);
  }

} // namespace cdd_access

} // namespace etsi_its_msgs
