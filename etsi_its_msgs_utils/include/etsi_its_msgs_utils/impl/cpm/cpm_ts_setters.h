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
 * @param confidence The confidence to be set in centimeters (default: infinity, which will map to CoordinateConfidence::UNAVAILABLE).
 */
inline void setCartesianCoordinateWithConfidence(CartesianCoordinateWithConfidence& coordinate, const double value,
                                                 const double confidence = std::numeric_limits<double>::infinity()) {
  // limit value range
  if (value < CartesianCoordinateLarge::NEGATIVE_OUT_OF_RANGE) {
    coordinate.value.value = CartesianCoordinateLarge::NEGATIVE_OUT_OF_RANGE;
  } else if (value > CartesianCoordinateLarge::POSITIVE_OUT_OF_RANGE) {
    coordinate.value.value = CartesianCoordinateLarge::POSITIVE_OUT_OF_RANGE;
  } else {
    coordinate.value.value = static_cast<int32_t>(value);
  }

  // limit confidence range
  if (confidence == std::numeric_limits<double>::infinity()) {
    coordinate.confidence.value = CoordinateConfidence::UNAVAILABLE;
  } else if (confidence > CoordinateConfidence::MAX || confidence < CoordinateConfidence::MIN) {
    coordinate.confidence.value = CoordinateConfidence::OUT_OF_RANGE;
  } else {
    coordinate.confidence.value = static_cast<uint16_t>(confidence);
  }
}

/**
 * @brief Sets the position of a perceived object (relative to the CPM's reference position).
 *
 * This function sets the position of a perceived object using the provided coordinates and confidence values.
 *
 * @param object The PerceivedObject to set the position for.
 * @param point The gm::Point representing the coordinates of the object in meters.
 * @param x_std The standard deviation in meters for the x-coordinate (default: infinity).
 * @param y_std The standard deviation in meters for the y-coordinate (default: infinity).
 * @param z_std The standard deviation in meters for the z-coordinate (default: infinity).
 */
inline void setPositionOfPerceivedObject(PerceivedObject& object, const gm::Point& point,
                                         const double x_std = std::numeric_limits<double>::infinity(),
                                         const double y_std = std::numeric_limits<double>::infinity(),
                                         const double z_std = std::numeric_limits<double>::infinity()) {
  setCartesianCoordinateWithConfidence(object.position.x_coordinate, point.x * 100, x_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  setCartesianCoordinateWithConfidence(object.position.y_coordinate, point.y * 100, y_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  if (point.z != 0.0) {
    setCartesianCoordinateWithConfidence(object.position.z_coordinate, point.z * 100, z_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
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
 * @param x_confidence The standard deviation in meters for the x coordinate (default: CoordinateConfidence::UNAVAILABLE).
 * @param y_confidence The standard deviation in meters for the y coordinate (default: CoordinateConfidence::UNAVAILABLE).
 * @param z_confidence The standard deviation in meters for the z coordinate (default: CoordinateConfidence::UNAVAILABLE).
 *
 * @throws std::invalid_argument if the UTM-Position frame_id does not match the reference position frame_id.
 */
inline void setUTMPositionOfPerceivedObject(CollectivePerceptionMessage& cpm, PerceivedObject& object,
                                            const gm::PointStamped& utm_position,
                                            const double x_std = std::numeric_limits<double>::infinity(),
                                            const double y_std = std::numeric_limits<double>::infinity(),
                                            const double z_std = std::numeric_limits<double>::infinity()) {
  gm::PointStamped reference_position = getUTMPosition(cpm);
  if (utm_position.header.frame_id != reference_position.header.frame_id) {
    throw std::invalid_argument("UTM-Position frame_id (" + utm_position.header.frame_id +
                                ") does not match the reference position frame_id (" + reference_position.header.frame_id +
                                ")");
  }
  setCartesianCoordinateWithConfidence(object.position.x_coordinate,
                                       (utm_position.point.x - reference_position.point.x) * 100, x_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  setCartesianCoordinateWithConfidence(object.position.y_coordinate,
                                       (utm_position.point.y - reference_position.point.y) * 100, y_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  if (utm_position.point.z != 0.0) {
    setCartesianCoordinateWithConfidence(object.position.z_coordinate,
                                         (utm_position.point.z - reference_position.point.z) * 100, z_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
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
 * @param confidence The confidence to set for the VelocityComponent in cm/s. Default value is infinity, mapping to SpeedConfidence::UNAVAILABLE.
 */
inline void setVelocityComponent(VelocityComponent& velocity, const double value,
                                 const double confidence = std::numeric_limits<double>::infinity()) {
  // limit value range
  if (value < VelocityComponentValue::NEGATIVE_OUT_OF_RANGE) {
    velocity.value.value = VelocityComponentValue::NEGATIVE_OUT_OF_RANGE;
  } else if (value > VelocityComponentValue::POSITIVE_OUT_OF_RANGE) {
    velocity.value.value = VelocityComponentValue::POSITIVE_OUT_OF_RANGE;
  } else {
    velocity.value.value = static_cast<int16_t>(value);
  }

  // limit confidence range
  if(confidence == std::numeric_limits<double>::infinity()) {
    velocity.confidence.value = SpeedConfidence::UNAVAILABLE;
  } else if (confidence > SpeedConfidence::MAX || confidence < SpeedConfidence::MIN) {
    velocity.confidence.value = SpeedConfidence::OUT_OF_RANGE;
  } else {
    velocity.confidence.value = static_cast<uint8_t>(confidence);
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
 * @param x_std The standard deviation in m/s for the x velocity component (default: infinity).
 * @param y_std The standard deviation in m/s for the y velocity component (default: infinity).
 * @param z_std The standard deviation in m/s for the z velocity component (default: infinity).
 */
inline void setVelocityOfPerceivedObject(PerceivedObject& object, const gm::Vector3& cartesian_velocity,
                                         const double x_std = std::numeric_limits<double>::infinity(),
                                         const double y_std = std::numeric_limits<double>::infinity(),
                                         const double z_std = std::numeric_limits<double>::infinity()) {
  object.velocity.choice = Velocity3dWithConfidence::CHOICE_CARTESIAN_VELOCITY;
  setVelocityComponent(object.velocity.cartesian_velocity.x_velocity, cartesian_velocity.x * 100, x_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  setVelocityComponent(object.velocity.cartesian_velocity.y_velocity, cartesian_velocity.y * 100, y_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  if (cartesian_velocity.z != 0.0) {
    setVelocityComponent(object.velocity.cartesian_velocity.z_velocity, cartesian_velocity.z * 100, z_std * 100 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
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
 * @param confidence The confidence to set for the AccelerationComponent in dm/s^2. Default value is infinity, mapping to AccelerationConfidence::UNAVAILABLE.
 */
inline void setAccelerationComponent(AccelerationComponent& acceleration, const double value,
                                     const double confidence = std::numeric_limits<double>::infinity()) {
  // limit value range
  if (value < AccelerationValue::NEGATIVE_OUT_OF_RANGE) {
    acceleration.value.value = AccelerationValue::NEGATIVE_OUT_OF_RANGE;
  } else if (value > AccelerationValue::POSITIVE_OUT_OF_RANGE) {
    acceleration.value.value = AccelerationValue::POSITIVE_OUT_OF_RANGE;
  } else {
    acceleration.value.value = static_cast<int16_t>(value);
  }

  // limit confidence range
  if(confidence == std::numeric_limits<double>::infinity()) {
    acceleration.confidence.value = AccelerationConfidence::UNAVAILABLE;
  } else if (confidence > AccelerationConfidence::MAX || confidence < AccelerationConfidence::MIN) {
    acceleration.confidence.value = AccelerationConfidence::OUT_OF_RANGE;
  } else {
    acceleration.confidence.value = static_cast<uint8_t>(confidence);
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
 * @param x_std The standard deviation in m/s^2 for the x acceleration component (default: infinity).
 * @param y_std The standard deviation in m/s^2 for the y acceleration component (default: infinity).
 * @param z_std The standard deviation in m/s^2 for the z acceleration component (default: infinity).
 */
inline void setAccelerationOfPerceivedObject(PerceivedObject& object, const gm::Vector3& cartesian_acceleration,
                                             const double x_std = std::numeric_limits<double>::infinity(),
                                             const double y_std = std::numeric_limits<double>::infinity(),
                                             const double z_std = std::numeric_limits<double>::infinity()) {
  object.acceleration.choice = Acceleration3dWithConfidence::CHOICE_CARTESIAN_ACCELERATION;
  setAccelerationComponent(object.acceleration.cartesian_acceleration.x_acceleration, cartesian_acceleration.x * 10,
                           x_std * 10 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  setAccelerationComponent(object.acceleration.cartesian_acceleration.y_acceleration, cartesian_acceleration.y * 10,
                           y_std * 10 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  if (cartesian_acceleration.z != 0.0) {
    setAccelerationComponent(object.acceleration.cartesian_acceleration.z_acceleration, cartesian_acceleration.z * 10,
                             z_std * 10 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
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
 * @param yaw_std The standard deviation of the yaw angle in radians (optional, default is Infinity).
 */
inline void setYawOfPerceivedObject(PerceivedObject& object, const double yaw,
                                    double yaw_std = std::numeric_limits<double>::infinity()) {
  // wrap angle to range [0, 360)
  double yaw_in_degrees = yaw * 180 / M_PI;
  while (yaw_in_degrees >= 360.0) yaw_in_degrees -= 360.0;
  while (yaw_in_degrees < 0.0) yaw_in_degrees += 360.0;
  object.angles.z_angle.value.value = yaw_in_degrees * 10;

  if(yaw_std == std::numeric_limits<double>::infinity()) {
    object.angles.z_angle.confidence.value = AngleConfidence::UNAVAILABLE;
  } else {
    yaw_std *= 180.0 / M_PI;
    yaw_std *= 10 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;  // convert to 0.1 degrees

    if (yaw_std > AngleConfidence::MAX || yaw_std < AngleConfidence::MIN) {
      object.angles.z_angle.confidence.value = AngleConfidence::OUT_OF_RANGE;
    } else {
      object.angles.z_angle.confidence.value = static_cast<uint8_t>(yaw_std);
    }
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
 * @param yaw_rate_std Standard deviation of the yaw rate in rad/s (optional, default is infinity, mapping to AngularSpeedConfidence::UNAVAILABLE).
 */
inline void setYawRateOfPerceivedObject(PerceivedObject& object, const double yaw_rate,
                                        double yaw_rate_std = std::numeric_limits<double>::infinity()) {
  double yaw_rate_in_degrees = yaw_rate * 180.0 / M_PI;
  // limit value range
  if (yaw_rate_in_degrees < CartesianAngularVelocityComponentValue::NEGATIVE_OUTOF_RANGE) {
    yaw_rate_in_degrees = CartesianAngularVelocityComponentValue::NEGATIVE_OUTOF_RANGE;
  } else if (yaw_rate_in_degrees > CartesianAngularVelocityComponentValue::POSITIVE_OUT_OF_RANGE) {
    yaw_rate_in_degrees = CartesianAngularVelocityComponentValue::POSITIVE_OUT_OF_RANGE;
  }
  
  object.z_angular_velocity.value.value = yaw_rate_in_degrees;

  if(yaw_rate_std == std::numeric_limits<double>::infinity()) {
    object.z_angular_velocity.confidence.value = AngularSpeedConfidence::UNAVAILABLE;
  } else {
    yaw_rate_std *= 180.0 / M_PI;
    yaw_rate_std *= etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;  // convert to degrees/s
    // How stupid is this?!
    static const std::map<double, uint8_t> confidence_map = {
        {1.0, AngularSpeedConfidence::DEG_SEC_01},
        {2.0, AngularSpeedConfidence::DEG_SEC_02},
        {5.0, AngularSpeedConfidence::DEG_SEC_05},
        {10.0, AngularSpeedConfidence::DEG_SEC_10},
        {20.0, AngularSpeedConfidence::DEG_SEC_20},
        {50.0, AngularSpeedConfidence::DEG_SEC_50},
        {std::numeric_limits<double>::infinity(), AngularSpeedConfidence::OUT_OF_RANGE},
    };
    for(const auto& [thresh, conf_val] : confidence_map) {
      if (yaw_rate_std <= thresh) {
        object.z_angular_velocity.confidence.value = conf_val;
        break;
      }
    }
  }

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
 * @param confidence The confidence of the object dimension in decimeters (optional, default is infinty, mapping to ObjectDimensionConfidence::UNAVAILABLE).
 */
inline void setObjectDimension(ObjectDimension& dimension, const double value,
                               const double confidence = std::numeric_limits<double>::infinity()) {
  // limit value range
  if (value < ObjectDimensionValue::MIN || value > ObjectDimensionValue::MAX) {
    dimension.value.value = ObjectDimensionValue::OUT_OF_RANGE;
  } else {
    dimension.value.value = static_cast<uint16_t>(value);
  }

  // limit confidence range
  if (confidence == std::numeric_limits<double>::infinity()) {
    dimension.confidence.value = ObjectDimensionConfidence::UNAVAILABLE;
  } else   if (confidence > ObjectDimensionConfidence::MAX || confidence < ObjectDimensionConfidence::MIN) {
    dimension.confidence.value = ObjectDimensionConfidence::OUT_OF_RANGE;
  } else {
    dimension.confidence.value = static_cast<uint8_t>(confidence);
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
 * @param std The standard deviation of the x-dimension value in meters (optional, default is infinity).
 */
inline void setXDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const double std = std::numeric_limits<double>::infinity()) {
  setObjectDimension(object.object_dimension_x, value * 10, std * 10 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
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
 * @param std The standard deviation of the y-dimension value in meters (optional, default is infinity).
 */
inline void setYDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const double std = std::numeric_limits<double>::infinity()) {
  setObjectDimension(object.object_dimension_y, value * 10, std * 10 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
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
 * @param std The standard deviation of the z-dimension value in meters (optional, default is infinity).
 */
inline void setZDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const double std = std::numeric_limits<double>::infinity()) {
  setObjectDimension(object.object_dimension_z, value * 10, std * 10 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  object.object_dimension_z_is_present = true;
}

/**
 * @brief Sets all dimensions of a perceived object.
 *
 * This function sets the dimensions of a perceived object using the provided dimensions and confidence values.
 *
 * @param object The perceived object to set the dimensions for.
 * @param dimensions The dimensions of the object as a gm::Vector3 (x, y, z) in meters.
 * @param x_std The standard deviation in meters for the x dimension (optional, default: infinity).
 * @param y_std The standard deviation in meters for the y dimension (optional, default: infinity).
 * @param z_std The standard deviation in meters for the z dimension (optional, default: infinity).
 */
inline void setDimensionsOfPerceivedObject(PerceivedObject& object, const gm::Vector3& dimensions,
                                           const double x_std = std::numeric_limits<double>::infinity(),
                                           const double y_std = std::numeric_limits<double>::infinity(),
                                           const double z_std = std::numeric_limits<double>::infinity()) {
  setXDimensionOfPerceivedObject(object, dimensions.x, x_std);
  setYDimensionOfPerceivedObject(object, dimensions.y, y_std);
  setZDimensionOfPerceivedObject(object, dimensions.z, z_std);
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

/**
 * @brief Set the confidence of the reference position
 * 
 * @param cpm CPM-Message to set the confidence
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is WGS84 (x = North, y = East)
 */
inline void setWGSRefPosConfidence(CollectivePerceptionMessage& cpm, const std::array<double, 4>& covariance_matrix) {
  setWGSPosConfidenceEllipse(cpm.payload.management_container.reference_position.position_confidence_ellipse,
                             covariance_matrix);
}

/**
 * @brief Initializes a WrappedCpmContainer as a SensorInformationContainer.
 *
 * This function sets the container ID to CHOICE_CONTAINER_DATA_SENSOR_INFORMATION_CONTAINER and initializes
 * an empty vector with SensorInformation.
 *
 * @param container A reference to the WrappedCpmContainer to be initialized as a SensorInformationContainer.
 */
inline void initSensorInformationContainer(WrappedCpmContainer& container) {
  container.container_id.value = WrappedCpmContainer::CHOICE_CONTAINER_DATA_SENSOR_INFORMATION_CONTAINER;
  container.container_data_sensor_information_container.array = std::vector<SensorInformation>();
}

/**
 * @brief Sets the sensorId of a SensorInformation object.
 *
 * This function sets the sensorId of a SensorInformation object. The sensorId is limited to the range defined by
 * Identifier1B::MIN and Identifier1B::MAX.
 *
 * @param sensor_information The SensorInformation object to set the sensorId for.
 * @param sensor_id The sensorId to set (default: 0).
 * @throws std::invalid_argument if the sensor_id is out of range.
 */
inline void setSensorID(SensorInformation& sensor_information, const uint8_t sensor_id = 0) {
  throwIfOutOfRange(sensor_id, Identifier1B::MIN, Identifier1B::MAX, "SensorID");
  sensor_information.sensor_id.value = sensor_id;
}

/**
 * @brief Sets the sensorType of a SensorInformation object.
 *
 * This function sets the sensorType of a SensorInformation object. The sensorType is limited to the range defined by
 * SensorType::MIN and SensorType::MAX.
 *
 * @param sensor_information The SensorInformation object to set the sensorType for.
 * @param sensor_type The sensorType to set (default: SensorType::UNDEFINED).
 * @throws std::invalid_argument if the sensor_type is out of range.
 */
inline void setSensorType(SensorInformation& sensor_information, const uint8_t sensor_type = SensorType::UNDEFINED) {
  throwIfOutOfRange(sensor_type, SensorType::MIN, SensorType::MAX, "SensorType");
  sensor_information.sensor_type.value = sensor_type;
}

/**
 * @brief Adds a SensorInformation to the SensorInformationContainer / WrappedCpmContainer.
 *
 * This function checks if the provided container is a SensorInformationContainer. If it is,
 * the function adds the given SensorInformation to the container's sensor_information array.
 * If the container is not a SensorInformationContainer, the function throws an
 * std::invalid_argument exception. If the maximum number of SensorInformation entries is reached,
 * it throws an exception. If the sensor_id or sensor_type is out of range, it throws an
 * exception.
 *
 *
 * @param container The WrappedCpmContainer to which the SensorInformation will be added.
 * @param sensor_information The SensorInformation to add to the container.
 *
 * @throws std::invalid_argument if the container is not a SensorInformationContainer.
 * @throws std::invalid_argument if the maximum number of SensorInformation entries is reached.
 * @throws std::invalid_argument if the sensor_id or sensor_type is out of range.
 */
inline void addSensorInformationToContainer(WrappedCpmContainer& container, const SensorInformation& sensor_information) {
  if (container.container_id.value == WrappedCpmContainer::CHOICE_CONTAINER_DATA_SENSOR_INFORMATION_CONTAINER) {
    // check for maximum number of SensorInformation entries
    if (container.container_data_sensor_information_container.array.size() < SensorInformationContainer::MAX_SIZE ) {
      throwIfOutOfRange(sensor_information.sensor_id.value, Identifier1B::MIN, Identifier1B::MAX, "SensorID");
      throwIfOutOfRange(sensor_information.sensor_type.value, SensorType::MIN, SensorType::MAX, "SensorType");
      container.container_data_sensor_information_container.array.push_back(sensor_information);
    } else {
      throw std::invalid_argument("Maximum number of entries SensorInformationContainers reached");
    }
  } else {
    throw std::invalid_argument("Container is not a SensorInformationContainer");
  }
}

/**
 * @brief Adds a container to the Collective Perception Message (CPM).
 *
 * This function adds a SensorInformationContainer / WrappedCpmContainer to the CPM's payload.
 * It first checks if the container is a SensorInformationContainer. If so, it checks if
 * the number of entries is in the allowed range. If the number of entries is valid,
 * it appends the container to the array. Otherwise, it throws an exception.
 *
 * @param cpm The Collective Perception Message to which the container will be added.
 * @param container The WrappedCpmContainer to be added to the CPM.
 *
 * @throws std::invalid_argument if the container is not a SensorInformationContainer.
 * @throws std::invalid_argument if the number of SensorInformation entries is out of range.
 */
inline void addSensorInformationContainerToCPM(CollectivePerceptionMessage& cpm, const WrappedCpmContainer& container) {
  if (container.container_id.value == WrappedCpmContainer::CHOICE_CONTAINER_DATA_SENSOR_INFORMATION_CONTAINER) {
    throwIfOutOfRange(container.container_data_sensor_information_container.array.size(),
      SensorInformationContainer::MIN_SIZE, SensorInformationContainer::MAX_SIZE, "SensorInformationContainer array size"
    );
    addContainerToCPM(cpm, container);
  } else {
    throw std::invalid_argument("Container is not a SensorInformationContainer");
  }
}

}  // namespace etsi_its_cpm_ts_msgs::access
