/*
=============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

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
 * @file impl/cpm/cpm_ts_setters.h
 * @brief Setter functions for the ETSI ITS CPM (TS)
 */

#pragma once

#include <etsi_its_msgs_utils/impl/checks.h>
#include <etsi_its_msgs_utils/impl/constants.h>

namespace etsi_its_cpm_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-1-1_setters.h>

/**
 * @brief Sets the ITS PDU header of a CPM.
 *
 * This function sets the ITS PDU header of a CPM with the provided station ID and protocol version.
 *
 * @param cpm The CPM to set the ITS PDU header for.
 * @param station_id The station ID to set in the ITS PDU header.
 * @param protocol_version The protocol version to set in the ITS PDU header. Default is 0.
 */
inline void setItsPduHeader(CollectivePerceptionMessage& cpm, const uint32_t station_id,
                            const uint8_t protocol_version = 0) {
  setItsPduHeader(cpm.header, MessageId::CPM, station_id, protocol_version);
}

/**
 * @brief Sets the reference time in a CPM.
 *
 * This function sets the reference time in a CPM object. The reference time is represented
 * by a Unix timestamp in nanoseconds including the number of leap seconds.
 * The reference time is stored in the payload management container of the CPM.
 *
 * @param cpm The CPM object to set the reference time in.
 * @param unix_nanosecs The Unix timestamp in nanoseconds representing the reference time.
 * @param n_leap_seconds The number of leap seconds to be considered. Defaults to the todays number of leap seconds since 2004.
 */
inline void setReferenceTime(
    CollectivePerceptionMessage& cpm, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  TimestampIts t_its;
  setTimestampITS(t_its, unix_nanosecs, n_leap_seconds);
  throwIfOutOfRange(t_its.value, TimestampIts::MIN, TimestampIts::MAX, "TimestampIts");
  cpm.payload.management_container.reference_time = t_its;
}

/**
 * @brief Set the ReferencePositionWithConfidence for a CPM TS
 *
 * This function sets the latitude, longitude, and altitude of the CPMs reference position.
 * If the altitude is not provided, it is set to AltitudeValue::UNAVAILABLE.
 *
 * @param cpm CPM to set the ReferencePosition
 * @param latitude The latitude value position in degree as decimal number.
 * @param longitude The longitude value in degree as decimal number.
 * @param altitude The altitude value (above the reference ellipsoid surface) in meter as decimal number (optional).
 */
inline void setReferencePosition(CollectivePerceptionMessage& cpm, const double latitude, const double longitude,
                                 const double altitude = AltitudeValue::UNAVAILABLE) {
  setReferencePosition(cpm.payload.management_container.reference_position, latitude, longitude, altitude);
}

/**
 * @brief Set the ReferencePosition of a CPM from a given UTM-Position
 *
 * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
 * The z-Coordinate is directly used as altitude value
 * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
 *
 * @param[out] cpm CPM for which to set the ReferencePosition
 * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position in meters
 * @param[in] zone the UTM zone (zero means UPS) of the given position
 * @param[in] northp hemisphere (true means north, false means south)
 */
inline void setFromUTMPosition(CollectivePerceptionMessage& cpm, const gm::PointStamped& utm_position, const int& zone,
                               const bool& northp) {
  setFromUTMPosition(cpm.payload.management_container.reference_position, utm_position, zone, northp);
}

/**
 * @brief Set the ID of a PerceivedObject
 *
 * Sets the object_id of the PerceivedObject to the given ID.
 *
 * @param object PerceivedObject to set the ID for
 * @param id ID to set
 */
inline void setIdOfPerceivedObject(PerceivedObject& object, const uint16_t id) {
  object.object_id.value = id;
  object.object_id_is_present = true;
}

/**
 * @brief Sets the measurement delta time of a PerceivedObject.
 * 
 * This function sets the measurement delta time of a PerceivedObject.
 * The measurement delta time represents the time difference 
 * between the reference time of the CPM and the measurement of the object.
 * 
 * @param object The PerceivedObject to set the measurement delta time for.
 * @param delta_time The measurement delta time to set in milliseconds. Default value is 0.
 * @throws std::invalid_argument if the delta_time is out of range.
 */
inline void setMeasurementDeltaTimeOfPerceivedObject(PerceivedObject& object, const int16_t delta_time = 0) {
  if (delta_time < DeltaTimeMilliSecondSigned::MIN || delta_time > DeltaTimeMilliSecondSigned::MAX) {
    throw std::invalid_argument("MeasurementDeltaTime out of range");
  } else {
    object.measurement_delta_time.value = delta_time;
  }
}

/**
 * @brief Sets the value and confidence of a CartesianCoordinateWithConfidence object.
 * 
 * This function sets the value and confidence of a CartesianCoordinateWithConfidence object.
 * The value is limited to the range defined by CartesianCoordinateLarge::NEGATIVE_OUT_OF_RANGE
 * and CartesianCoordinateLarge::POSITIVE_OUT_OF_RANGE. The confidence is limited to the range
 * defined by CoordinateConfidence::MIN and CoordinateConfidence::MAX. If the provided confidence
 * is out of range, the confidence value is set to CoordinateConfidence::OUT_OF_RANGE.
 * 
 * @param coordinate The CartesianCoordinateWithConfidence object to be modified.
 * @param value The value to be set in centimeters.
 * @param confidence The confidence to be set in centimeters (default: CoordinateConfidence::UNAVAILABLE).
 */
inline void setCartesianCoordinateWithConfidence(CartesianCoordinateWithConfidence& coordinate, const int32_t value,
                                                 const uint16_t confidence = CoordinateConfidence::UNAVAILABLE) {
  // limit value range
  if (value < CartesianCoordinateLarge::NEGATIVE_OUT_OF_RANGE) {
    coordinate.value.value = CartesianCoordinateLarge::NEGATIVE_OUT_OF_RANGE;
  } else if (value > CartesianCoordinateLarge::POSITIVE_OUT_OF_RANGE) {
    coordinate.value.value = CartesianCoordinateLarge::POSITIVE_OUT_OF_RANGE;
  } else {
    coordinate.value.value = value;
  }

  // limit confidence range
  if (confidence > CoordinateConfidence::MAX || confidence < CoordinateConfidence::MIN) {
    coordinate.confidence.value = CoordinateConfidence::OUT_OF_RANGE;
  } else {
    coordinate.confidence.value = confidence;
  }
}

/**
 * @brief Sets the position of a perceived object (relative to the CPM's reference position).
 *
 * This function sets the position of a perceived object using the provided coordinates and confidence values.
 *
 * @param object The PerceivedObject to set the position for.
 * @param point The gm::Point representing the coordinates of the object in meters.
 * @param x_confidence The confidence value in meters for the x-coordinate (default: CoordinateConfidence::UNAVAILABLE).
 * @param y_confidence The confidence value in meters for the y-coordinate (default: CoordinateConfidence::UNAVAILABLE).
 * @param z_confidence The confidence value in meters for the z-coordinate (default: CoordinateConfidence::UNAVAILABLE).
 */
inline void setPositionOfPerceivedObject(PerceivedObject& object, const gm::Point& point,
                                         const uint16_t x_confidence = CoordinateConfidence::UNAVAILABLE,
                                         const uint16_t y_confidence = CoordinateConfidence::UNAVAILABLE,
                                         const uint16_t z_confidence = CoordinateConfidence::UNAVAILABLE) {
  setCartesianCoordinateWithConfidence(object.position.x_coordinate, point.x * 100, x_confidence * 100);
  setCartesianCoordinateWithConfidence(object.position.y_coordinate, point.y * 100, y_confidence * 100);
  if (point.z != 0.0) {
    setCartesianCoordinateWithConfidence(object.position.z_coordinate, point.z * 100, z_confidence * 100);
    object.position.z_coordinate_is_present = true;
  }
}

/**
 * @brief Sets the position of a perceived object based on a UTM position.
 *
 * This function sets the position of a perceived object using the provided UTM position and the CPM's reference position.
 * It also allows specifying confidence values for the x, y, and z coordinates.
 *
 * @param cpm The CPM to get the reference position from.
 * @param object The PerceivedObject to set the position for.
 * @param utm_position gm::PointStamped representing the UTM position of the object including the frame_id (utm_<zone><N/S>).
 * @param x_confidence The confidence value in meters for the x coordinate (default: CoordinateConfidence::UNAVAILABLE).
 * @param y_confidence The confidence value in meters for the y coordinate (default: CoordinateConfidence::UNAVAILABLE).
 * @param z_confidence The confidence value in meters for the z coordinate (default: CoordinateConfidence::UNAVAILABLE).
 *
 * @throws std::invalid_argument if the UTM-Position frame_id does not match the reference position frame_id.
 */
inline void setUTMPositionOfPerceivedObject(CollectivePerceptionMessage& cpm, PerceivedObject& object,
                                            const gm::PointStamped& utm_position,
                                            const uint16_t x_confidence = CoordinateConfidence::UNAVAILABLE,
                                            const uint16_t y_confidence = CoordinateConfidence::UNAVAILABLE,
                                            const uint16_t z_confidence = CoordinateConfidence::UNAVAILABLE) {
  gm::PointStamped reference_position = getUTMPosition(cpm);
  if (utm_position.header.frame_id != reference_position.header.frame_id) {
    throw std::invalid_argument("UTM-Position frame_id (" + utm_position.header.frame_id +
                                ") does not match the reference position frame_id (" + reference_position.header.frame_id +
                                ")");
  }
  setCartesianCoordinateWithConfidence(object.position.x_coordinate,
                                       (utm_position.point.x - reference_position.point.x) * 100, x_confidence);
  setCartesianCoordinateWithConfidence(object.position.y_coordinate,
                                       (utm_position.point.y - reference_position.point.y) * 100, y_confidence);
  if (utm_position.point.z != 0.0) {
    setCartesianCoordinateWithConfidence(object.position.z_coordinate,
                                         (utm_position.point.z - reference_position.point.z) * 100, z_confidence);
    object.position.z_coordinate_is_present = true;
  }
}

/**
 * @brief Sets the value and confidence of a VelocityComponent.
 *
 * This function sets the value and confidence of a VelocityComponent object. The value is limited to a specific range,
 * and the confidence is limited to a specific range as well. If the provided value or confidence is out of range,
 * it will be set to the corresponding out-of-range value.
 *
 * @param velocity The VelocityComponent object to set the value and confidence for.
 * @param value The value to set for the VelocityComponent in cm/s.
 * @param confidence The confidence to set for the VelocityComponent in cm/s. Default value is SpeedConfidence::UNAVAILABLE.
 */
inline void setVelocityComponent(VelocityComponent& velocity, const int16_t value,
                                 const uint8_t confidence = SpeedConfidence::UNAVAILABLE) {
  // limit value range
  if (value < VelocityComponentValue::NEGATIVE_OUT_OF_RANGE) {
    velocity.value.value = VelocityComponentValue::NEGATIVE_OUT_OF_RANGE;
  } else if (value > VelocityComponentValue::POSITIVE_OUT_OF_RANGE) {
    velocity.value.value = VelocityComponentValue::POSITIVE_OUT_OF_RANGE;
  } else {
    velocity.value.value = value;
  }

  // limit confidence range
  if (confidence > SpeedConfidence::MAX || confidence < SpeedConfidence::MIN) {
    velocity.confidence.value = SpeedConfidence::OUT_OF_RANGE;
  } else {
    velocity.confidence.value = confidence;
  }
}

/**
 * Sets the velocity of a perceived object.
 *
 * This function sets the velocity of a perceived object using the provided Cartesian velocity components.
 * Optionally, confidence values can be specified for each velocity component.
 *
 * @param object The perceived object for which the velocity is being set.
 * @param cartesian_velocity The Cartesian velocity components (x, y, z) of the object (in m/s).
 * @param x_confidence The confidence value in m/s for the x velocity component (default: SpeedConfidence::UNAVAILABLE).
 * @param y_confidence The confidence value in m/s for the y velocity component (default: SpeedConfidence::UNAVAILABLE).
 * @param z_confidence The confidence value in m/s for the z velocity component (default: SpeedConfidence::UNAVAILABLE).
 */
inline void setVelocityOfPerceivedObject(PerceivedObject& object, const gm::Vector3& cartesian_velocity,
                                         const uint8_t x_confidence = SpeedConfidence::UNAVAILABLE,
                                         const uint8_t y_confidence = SpeedConfidence::UNAVAILABLE,
                                         const uint8_t z_confidence = SpeedConfidence::UNAVAILABLE) {
  object.velocity.choice = Velocity3dWithConfidence::CHOICE_CARTESIAN_VELOCITY;
  setVelocityComponent(object.velocity.cartesian_velocity.x_velocity, cartesian_velocity.x * 100, x_confidence * 100);
  setVelocityComponent(object.velocity.cartesian_velocity.y_velocity, cartesian_velocity.y * 100, y_confidence * 100);
  if (cartesian_velocity.z != 0.0) {
    setVelocityComponent(object.velocity.cartesian_velocity.z_velocity, cartesian_velocity.z * 100, z_confidence * 100);
    object.velocity.cartesian_velocity.z_velocity_is_present = true;
  }
  object.velocity_is_present = true;
}

/**
 * @brief Sets the value and confidence of a AccelerationComponent.
 *
 * This function sets the value and confidence of a AccelerationComponent object. The value is limited to a specific range,
 * and the confidence is limited to a specific range as well. If the provided value or confidence is out of range,
 * it will be set to the corresponding out-of-range value.
 *
 * @param acceleration The AccelerationComponent object to set the value and confidence for.
 * @param value The value to set for the AccelerationComponent in dm/s^2.
 * @param confidence The confidence to set for the AccelerationComponent in dm/s^2. Default value is AccelerationConfidence::UNAVAILABLE.
 */
inline void setAccelerationComponent(AccelerationComponent& acceleration, const int16_t value,
                                     const uint8_t confidence = AccelerationConfidence::UNAVAILABLE) {
  // limit value range
  if (value < AccelerationValue::NEGATIVE_OUT_OF_RANGE) {
    acceleration.value.value = AccelerationValue::NEGATIVE_OUT_OF_RANGE;
  } else if (value > AccelerationValue::POSITIVE_OUT_OF_RANGE) {
    acceleration.value.value = AccelerationValue::POSITIVE_OUT_OF_RANGE;
  } else {
    acceleration.value.value = value;
  }

  // limit confidence range
  if (confidence > AccelerationConfidence::MAX || confidence < AccelerationConfidence::MIN) {
    acceleration.confidence.value = AccelerationConfidence::OUT_OF_RANGE;
  } else {
    acceleration.confidence.value = confidence;
  }
}

/**
 * @brief Sets the acceleration of a perceived object.
 *
 * This function sets the acceleration of a perceived object using the provided Cartesian acceleration components.
 * Optionally, confidence values can be specified for each acceleration component.
 *
 * @param object The perceived object for which the acceleration is being set.
 * @param cartesian_acceleration The Cartesian acceleration components (x, y, z) of the object (in m/s^2).
 * @param x_confidence The confidence value in m/s^2 for the x acceleration component (default: AccelerationConfidence::UNAVAILABLE).
 * @param y_confidence The confidence value in m/s^2 for the y acceleration component (default: AccelerationConfidence::UNAVAILABLE).
 * @param z_confidence The confidence value in m/s^2 for the z acceleration component (default: AccelerationConfidence::UNAVAILABLE).
 */
inline void setAccelerationOfPerceivedObject(PerceivedObject& object, const gm::Vector3& cartesian_acceleration,
                                             const uint8_t x_confidence = AccelerationConfidence::UNAVAILABLE,
                                             const uint8_t y_confidence = AccelerationConfidence::UNAVAILABLE,
                                             const uint8_t z_confidence = AccelerationConfidence::UNAVAILABLE) {
  object.acceleration.choice = Acceleration3dWithConfidence::CHOICE_CARTESIAN_ACCELERATION;
  setAccelerationComponent(object.acceleration.cartesian_acceleration.x_acceleration, cartesian_acceleration.x * 10,
                           x_confidence * 10);
  setAccelerationComponent(object.acceleration.cartesian_acceleration.y_acceleration, cartesian_acceleration.y * 10,
                           y_confidence * 10);
  if (cartesian_acceleration.z != 0.0) {
    setAccelerationComponent(object.acceleration.cartesian_acceleration.z_acceleration, cartesian_acceleration.z * 10,
                             z_confidence * 10);
    object.acceleration.cartesian_acceleration.z_acceleration_is_present = true;
  }
  object.acceleration_is_present = true;
}

/**
 * @brief Sets the yaw angle of a perceived object.
 *
 * This function sets the yaw angle of a PerceivedObject. The yaw angle is wrapped to the range [0, 360] degrees.
 * The function also allows specifying the confidence level of the yaw angle.
 *
 * @param object The PerceivedObject to set the yaw angle for.
 * @param yaw The yaw angle in radians.
 * @param confidence The confidence level of the yaw angle in 0,1 degrees (optional, default is AngleConfidence::UNAVAILABLE).
 */
inline void setYawOfPerceivedObject(PerceivedObject& object, const double yaw,
                                    const uint8_t confidence = AngleConfidence::UNAVAILABLE) {
  // wrap angle to range [0, 360]
  double yaw_in_degrees = yaw * 180 / M_PI + 180;  // TODO: check if this is correct
  while (yaw_in_degrees > 360.0) yaw_in_degrees -= 360.0;
  while (yaw_in_degrees < 0) yaw_in_degrees += 360.0;
  object.angles.z_angle.value.value = yaw_in_degrees * 10;

  if (confidence > AngleConfidence::MAX || confidence < AngleConfidence::MIN) {
    object.angles.z_angle.confidence.value = AngleConfidence::OUT_OF_RANGE;
  } else {
    object.angles.z_angle.confidence.value = confidence;
  }
  object.angles_is_present = true;
}

/**
 * @brief Sets the yaw rate of a perceived object.
 *
 * This function sets the yaw rate of a PerceivedObject. The yaw rate is limited to the range defined by
 * CartesianAngularVelocityComponentValue::NEGATIVE_OUTOF_RANGE and CartesianAngularVelocityComponentValue::POSITIVE_OUT_OF_RANGE.
 * The function also allows specifying the confidence level of the yaw rate.
 *
 * @param object The PerceivedObject to set the yaw rate for.
 * @param yaw_rate The yaw rate in rad/s.
 * @param confidence Confidence of the yaw rate defined in AngularSpeedConfidence (optional, default is AngularSpeedConfidence::UNAVAILABLE).
 */
inline void setYawRateOfPerceivedObject(PerceivedObject& object, const double yaw_rate,
                                        const uint8_t confidence = AngularSpeedConfidence::UNAVAILABLE) {
  int16_t yaw_rate_in_degrees = yaw_rate * 180 / M_PI;
  if (yaw_rate_in_degrees != CartesianAngularVelocityComponentValue::UNAVAILABLE) {
    // limit value range
    if (yaw_rate_in_degrees < CartesianAngularVelocityComponentValue::NEGATIVE_OUTOF_RANGE) {
      yaw_rate_in_degrees = CartesianAngularVelocityComponentValue::NEGATIVE_OUTOF_RANGE;
    } else if (yaw_rate_in_degrees > CartesianAngularVelocityComponentValue::POSITIVE_OUT_OF_RANGE) {
      yaw_rate_in_degrees = CartesianAngularVelocityComponentValue::POSITIVE_OUT_OF_RANGE;
    }
  }
  object.z_angular_velocity.value.value = yaw_rate_in_degrees;
  object.z_angular_velocity.confidence.value = confidence;
  object.z_angular_velocity_is_present = true;
}

/**
 * @brief Sets the object dimension with the given value and confidence.
 * 
 * This function sets the value and confidence of the object dimension based on the provided parameters.
 * The value is limited to the range defined by ObjectDimensionValue::MIN and ObjectDimensionValue::MAX.
 * If the provided value is outside this range, the dimension value is set to ObjectDimensionValue::OUT_OF_RANGE.
 * 
 * The confidence is limited to the range defined by ObjectDimensionConfidence::MIN and ObjectDimensionConfidence::MAX.
 * If the provided confidence is outside this range, the confidence value is set to ObjectDimensionConfidence::OUT_OF_RANGE.
 * 
 * @param dimension The object dimension to be set.
 * @param value The value of the object dimension in decimeters.
 * @param confidence The confidence of the object dimension in decimeters (optional, default is ObjectDimensionConfidence::UNAVAILABLE).
 */
inline void setObjectDimension(ObjectDimension& dimension, const uint16_t value,
                               const uint8_t confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  // limit value range
  if (value < ObjectDimensionValue::MIN || value > ObjectDimensionValue::MAX) {
    dimension.value.value = ObjectDimensionValue::OUT_OF_RANGE;
  } else {
    dimension.value.value = value;
  }

  // limit confidence range
  if (confidence > ObjectDimensionConfidence::MAX || confidence < ObjectDimensionConfidence::MIN) {
    dimension.confidence.value = ObjectDimensionConfidence::OUT_OF_RANGE;
  } else {
    dimension.confidence.value = confidence;
  }
}

/**
 * @brief Sets the x-dimension of a perceived object.
 *
 * This function sets the x-dimension of the given `PerceivedObject` to the specified value.
 * The x-dimension usually represents the length of the object.
 *
 * @param object The `PerceivedObject` to modify.
 * @param value The value to set as the x-dimension in meters.
 * @param confidence The confidence of the x-dimension value in meters (optional, default is `ObjectDimensionConfidence::UNAVAILABLE`).
 */
inline void setXDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const uint8_t confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  setObjectDimension(object.object_dimension_x, (uint16_t)(value * 10), confidence * 10);
  object.object_dimension_x_is_present = true;
}

/**
 * @brief Sets the y-dimension of a perceived object.
 *
 * This function sets the y-dimension of the given `PerceivedObject` to the specified value.
 * The y-dimension usually represents the width of the object.
 *
 * @param object The `PerceivedObject` to modify.
 * @param value The value to set as the y-dimension in meters.
 * @param confidence The confidence of the y-dimension value in meters (optional, default is `ObjectDimensionConfidence::UNAVAILABLE`).
 */
inline void setYDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const uint8_t confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  setObjectDimension(object.object_dimension_y, (uint16_t)(value * 10), confidence * 10);
  object.object_dimension_y_is_present = true;
}

/**
 * @brief Sets the z-dimension of a perceived object.
 *
 * This function sets the z-dimension of the given `PerceivedObject` to the specified value.
 * The z-dimension usually represents the height of the object.
 *
 * @param object The `PerceivedObject` to modify.
 * @param value The value to set as the z-dimension in meters.
 * @param confidence The confidence of the z-dimension value in meters (optional, default is `ObjectDimensionConfidence::UNAVAILABLE`).
 */
inline void setZDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const uint8_t confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  setObjectDimension(object.object_dimension_z, (uint16_t)(value * 10), confidence * 10);
  object.object_dimension_z_is_present = true;
}

/**
 * @brief Sets all dimensions of a perceived object.
 *
 * This function sets the dimensions of a perceived object using the provided dimensions and confidence values.
 *
 * @param object The perceived object to set the dimensions for.
 * @param dimensions The dimensions of the object as a gm::Vector3 (x, y, z) in meters.
 * @param x_confidence The confidence in meters for the x dimension (optional, default: ObjectDimensionConfidence::UNAVAILABLE).
 * @param y_confidence The confidence in meters for the y dimension (optional, default: ObjectDimensionConfidence::UNAVAILABLE).
 * @param z_confidence The confidence in meters for the z dimension (optional, default: ObjectDimensionConfidence::UNAVAILABLE).
 */
inline void setDimensionsOfPerceivedObject(PerceivedObject& object, const gm::Vector3& dimensions,
                                           const uint8_t x_confidence = ObjectDimensionConfidence::UNAVAILABLE,
                                           const uint8_t y_confidence = ObjectDimensionConfidence::UNAVAILABLE,
                                           const uint8_t z_confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  setXDimensionOfPerceivedObject(object, dimensions.x, x_confidence);
  setYDimensionOfPerceivedObject(object, dimensions.y, y_confidence);
  setZDimensionOfPerceivedObject(object, dimensions.z, z_confidence);
}

/**
 * @brief Initializes a PerceivedObject with the given point and delta time.
 *
 * This function sets the position and measurement delta time of the PerceivedObject.
 *
 * @param object The PerceivedObject to be initialized.
 * @param point The position of the PerceivedObject relative to the CPM's reference position in meters.
 * @param delta_time The measurement delta time of the PerceivedObject in milliseconds (default = 0).
 */
inline void initPerceivedObject(PerceivedObject& object, const gm::Point& point, const int16_t delta_time = 0) {
  setPositionOfPerceivedObject(object, point);
  setMeasurementDeltaTimeOfPerceivedObject(object, delta_time);
}

/**
 * @brief Initializes a PerceivedObject with the given point (utm-position) and delta time.
 *
 * This function initializes a PerceivedObject within a position and measurement delta time.
 * It sets the position of a perceived object using the provided UTM position and the CPM's reference position.
 * It sets the measurement delta time using the provided delta_time value.
 *
 * @param cpm The CPM to get the reference position from.
 * @param object The PerceivedObject to be initialized.
 * @param point The gm::PointStamped representing the UTM position of the object including the frame_id (utm_<zone><N/S>).
 * @param delta_time The measurement delta time in milliseconds (default: 0).
 */
inline void initPerceivedObjectWithUTMPosition(CollectivePerceptionMessage& cpm, PerceivedObject& object,
                                               const gm::PointStamped& point, const int16_t delta_time = 0) {
  setUTMPositionOfPerceivedObject(cpm, object, point);
  setMeasurementDeltaTimeOfPerceivedObject(object, delta_time);
}

/**
 * @brief Initializes a WrappedCpmContainer as a PerceivedObjectContainer with the given number of objects.
 *
 * This function sets the container ID to CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER and initializes
 * the number of perceived objects in the container to the specified value.
 *
 * @param container A reference to the WrappedCpmContainer to be initialized as a PerceivedObjectContainer.
 * @param n_objects The number of perceived objects to initialize in the container. Default is 0.
 */
inline void initPerceivedObjectContainer(WrappedCpmContainer& container, const uint8_t n_objects = 0) {
  container.container_id.value = WrappedCpmContainer::CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER;
  container.container_data_perceived_object_container.number_of_perceived_objects.value = n_objects;
}

/**
 * @brief Adds a PerceivedObject to the PerceivedObjectContainer / WrappedCpmContainer.
 * 
 * This function checks if the provided container is a PerceivedObjectContainer. If it is, 
 * the function adds the given PerceivedObject to the container's perceived_objects array 
 * and updates the number_of_perceived_objects value. If the container is not a 
 * PerceivedObjectContainer, the function throws an std::invalid_argument exception.
 * 
 * @param container The WrappedCpmContainer to which the PerceivedObject will be added.
 * @param perceived_object The PerceivedObject to add to the container.
 * 
 * @throws std::invalid_argument if the container is not a PerceivedObjectContainer.
 */
inline void addPerceivedObjectToContainer(WrappedCpmContainer& container, const PerceivedObject& perceived_object) {
  if (container.container_id.value == WrappedCpmContainer::CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER) {
    container.container_data_perceived_object_container.perceived_objects.array.push_back(perceived_object);
    container.container_data_perceived_object_container.number_of_perceived_objects.value =
        container.container_data_perceived_object_container.perceived_objects.array.size();
  } else {
    throw std::invalid_argument("Container is not a PerceivedObjectContainer");
  }
}

/**
 * @brief Adds a container to the Collective Perception Message (CPM).
 *
 * This function adds a WrappedCpmContainer to the CPM's payload. It first checks 
 * if the current number of containers is less than the maximum allowed. If so, 
 * it appends the container to the array. Otherwise, it throws an exception.
 *
 * @param cpm The Collective Perception Message to which the container will be added.
 * @param container The WrappedCpmContainer to be added to the CPM.
 * 
 * @throws std::invalid_argument if the maximum number of CPM-Containers is reached.
 */
inline void addContainerToCPM(CollectivePerceptionMessage& cpm, const WrappedCpmContainer& container) {
  // check for maximum number of containers
  if (cpm.payload.cpm_containers.value.array.size() < WrappedCpmContainers::MAX_SIZE) {
    cpm.payload.cpm_containers.value.array.push_back(container);
  } else {
    throw std::invalid_argument("Maximum number of CPM-Containers reached");
  }
}

}  // namespace etsi_its_cpm_ts_msgs::access
