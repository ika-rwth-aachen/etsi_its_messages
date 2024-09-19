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
 * @file impl/cpm/cpm_ts_setters.h
 * @brief Setter functions for the ETSI ITS CPM (TS)
 */

#pragma once

#include <etsi_its_msgs_utils/impl/constants.h>

namespace etsi_its_cpm_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-1-1_setters.h>

/**
   * @brief Set the ItsPduHeader-object for a CPM
   *
   * @param cpm CPM-Message to set the ItsPduHeader
   * @param station_id
   * @param protocol_version
   */
inline void setItsPduHeader(CollectivePerceptionMessage& cpm, const uint32_t station_id,
                            const uint8_t protocol_version = 0) {
  setItsPduHeader(cpm.header, MessageId::CPM, station_id, protocol_version);
}

/**
   * @brief Set the ReferenceTime-value
   * 
   * @param cpm CPM to set the ReferenceTime-Value for
   * @param unix_nanosecs Timestamp in unix-nanoseconds to set the ReferenceTime-Value from
   * @param n_leap_seconds Number of leap seconds since 2004 for the given timestamp  (Default: etsi_its_msgs::N_LEAP_SECONDS)
   */
inline void setReferenceTime(
    CollectivePerceptionMessage& cpm, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second) {
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
   * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
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
   * @brief Set the measurement_delta_time of a PerceivedObject
   *
   * Sets the object_type of the PerceivedObject to the given type.
   *
   * @param object PerceivedObject to set the type for
   * @param delta_time Delta time to set in milliseconds
   */
inline void setMeasurementDeltaTimeOfPerceivedObject(PerceivedObject& object, const int16_t delta_time = 0) {
  if (delta_time < DeltaTimeMilliSecondSigned::MIN || delta_time > DeltaTimeMilliSecondSigned::MAX) {
    throw std::invalid_argument("MeasurementDeltaTime out of range");
  } else {
    object.measurement_delta_time.value = delta_time;
  }
}

/**
   * @brief Set cartesian coordinate with confidence
   *
   * Sets the object_type of the PerceivedObject to the given type.
   *
   * @param coordinate CartesianCoordinateWithConfidence to set
   * @param value Value to set in cm
   * @param confidence Confidence to set cm (optional)
   */
inline void setCartesianCoordinateWithConfidence(CartesianCoordinateWithConfidence& coordinate, const int32_t value,
                                                 const uint16_t confidence = CoordinateConfidence::UNAVAILABLE) {
  // limit value range
  int32_t limited_value = std::max(CartesianCoordinateLarge::NEGATIVE_OUT_OF_RANGE,
                                   std::min(CartesianCoordinateLarge::POSITIVE_OUT_OF_RANGE, value));
  coordinate.value.value = limited_value;

  // limit confidence range
  if (confidence > CoordinateConfidence::MAX || confidence < CoordinateConfidence::MIN) {
    coordinate.confidence.value = CoordinateConfidence::OUT_OF_RANGE;
  } else {
    coordinate.confidence.value = confidence;
  }
}

/**
   * @brief Set the position of the PerceivedObject
   *
   * Sets the position of the PerceivedObject to the given point.
   * If the z-coordinate is not provided, it is set to 0.
   *
   * @param object PerceivedObject to set the position for
   * @param point Point to set the position to in m
   * @param x_confidence Confidence of the x-coordinate in m (optional)
   * @param y_confidence Confidence of the y-coordinate in m (optional)
   * @param z_confidence Confidence of the z-coordinate in m (optional)
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
   * @brief Set the position of the PerceivedObject in UTM
   *
   * Sets the position of the PerceivedObject to the given UTM position.
   * The UTM position is transformed to a relative position to the reference position of the CPM.
   *
   * @param cpm CPM to get the reference position from
   * @param object PerceivedObject to set the position for
   * @param utm_position UTM position to set the position to
   * @param x_confidence Confidence of the x-coordinate in m (optional)
   * @param y_confidence Confidence of the y-coordinate in m (optional)
   * @param z_confidence Confidence of the z-coordinate in m (optional)
   */
inline void setUTMPositionOfPerceivedObject(CollectivePerceptionMessage& cpm, PerceivedObject& object,
                                            const gm::PointStamped& utm_position,
                                            const uint16_t x_confidence = CoordinateConfidence::UNAVAILABLE,
                                            const uint16_t y_confidence = CoordinateConfidence::UNAVAILABLE,
                                            const uint16_t z_confidence = CoordinateConfidence::UNAVAILABLE) {
  gm::PointStamped reference_position = getUTMPosition(cpm);
  if (utm_position.header.frame_id != reference_position.header.frame_id) {
    throw std::invalid_argument("UTM-Position frame_id does not match the reference position frame_id");
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
   * @brief Set the orientation of the PerceivedObject
   *
   * Sets the orientation of the PerceivedObject to the given quaternion.
   * If the roll and pitch angles are not provided, they are set to 0.
   *
   * @param velocity velocity (x,y,z) component to set
   * @param value Value to set in cm/s
   * @param confidence Confidence to set in cm/s (optional)
   */
inline void setVelocityComponent(VelocityComponent& velocity, const int16_t value,
                                 const uint8_t confidence = SpeedConfidence::UNAVAILABLE) {
  // limit value range
  int16_t limited_value = std::max(VelocityComponentValue::NEGATIVE_OUT_OF_RANGE,
                                   std::min(VelocityComponentValue::POSITIVE_OUT_OF_RANGE, value));
  velocity.value.value = limited_value;

  // limit confidence range
  if (confidence > SpeedConfidence::MAX || confidence < SpeedConfidence::MIN) {
    velocity.confidence.value = SpeedConfidence::OUT_OF_RANGE;
  } else {
    velocity.confidence.value = confidence;
  }
}

/**
   * @brief Set the velocity of the PerceivedObject
   *
   * Sets the velocity of the PerceivedObject to the given velocity.
   * If the z-velocity is not provided, it is set to 0.
   *
   * @param object PerceivedObject to set the velocity for
   * @param cartesian_velocity Velocity to set in m/s
   * @param x_confidence Confidence of the x-velocity in m/s (optional)
   * @param y_confidence Confidence of the y-velocity in m/s (optional)
   * @param z_confidence Confidence of the z-velocity in m/s (optional)
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
   * @brief Set the acceleration component of the PerceivedObject
   *
   * Sets the acceleration component of the PerceivedObject to the given value.
   *
   * @param acceleration AccelerationComponent to set the value for
   * @param value Value to set in dm/s^2
   * @param confidence Confidence to set in dm/s^2 (optional)
   */
inline void setAccelerationComponent(AccelerationComponent& acceleration, const int16_t value,
                                     const uint8_t confidence = AccelerationConfidence::UNAVAILABLE) {
  // limit value range
  int16_t limited_value =
      std::max(AccelerationValue::NEGATIVE_OUT_OF_RANGE, std::min(AccelerationValue::POSITIVE_OUT_OF_RANGE, value));
  acceleration.value.value = limited_value;

  // limit confidence range
  if (confidence > AccelerationConfidence::MAX || confidence < AccelerationConfidence::MIN) {
    acceleration.confidence.value = AccelerationConfidence::OUT_OF_RANGE;
  } else {
    acceleration.confidence.value = confidence;
  }
}

/**
   * @brief Set the acceleration of the PerceivedObject
   *
   * Sets the acceleration of the PerceivedObject to the given acceleration.
   * If the z-acceleration is not provided, it is set to 0.
   *
   * @param object PerceivedObject to set the acceleration for
   * @param cartesian_acceleration Acceleration to set in m/s^2
   * @param x_confidence Confidence of the x-acceleration in m/s^2 (optional)
   * @param y_confidence Confidence of the y-acceleration in m/s^2 (optional)
   * @param z_confidence Confidence of the z-acceleration in m/s^2 (optional)
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
   * @brief Set the yaw angle of the PerceivedObject
   *
   * Sets the yaw angle of the PerceivedObject to the given angle.
   *
   * @param object PerceivedObject to set the roll angle for
   * @param roll Roll angle to set in rad
   * @param confidence Confidence of the roll angle in 0,1 degrees (optional)
   */
inline void setYawOfPerceivedObject(PerceivedObject& object, const double yaw,
                                    const uint8_t confidence = AngleConfidence::UNAVAILABLE) {
  // wrap angle to range [0, 360]
  double yaw_in_degrees = yaw * 180 / M_PI + 180;
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
   * @brief Set the yaw rate of the PerceivedObject
   *
   * Sets the yaw rate of the PerceivedObject
   *
   * @param object PerceivedObject to set the orientation for
   * @param yaw_rate Yaw rate to set in rad/s
   * @param confidence Confidence of the yaw rate in degrees/s (optional)
   */
inline void setYawRateOfPerceivedObject(PerceivedObject& object, const double yaw_rate,
                                        const uint8_t confidence = AngularSpeedConfidence::UNAVAILABLE) {
  int16_t yaw_rate_in_degrees = yaw_rate * 180 / M_PI;
  if (yaw_rate_in_degrees != CartesianAngularVelocityComponentValue::UNAVAILABLE) {
    yaw_rate_in_degrees =
        std::max(CartesianAngularVelocityComponentValue::NEGATIVE_OUTOF_RANGE,
                 std::min(CartesianAngularVelocityComponentValue::POSITIVE_OUT_OF_RANGE, yaw_rate_in_degrees));
  }
  object.z_angular_velocity.value.value = yaw_rate_in_degrees;
  object.z_angular_velocity.confidence.value = confidence;
  object.z_angular_velocity_is_present = true;
}

/**
   * @brief Set the dimension value to a given dimension
   *
   * Sets the dimension of the PerceivedObject to the given dimensions.
   *
   * @param dimension ObjectDimension to set the value for
   * @param value Value to set in cm
   * @param confidence Confidence of the value in cm (optional)
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
   * @brief Set the x-dimension of the PerceivedObject
   *
   * Sets the x-dimension of the PerceivedObject to the given value.
   *
   * @param object PerceivedObject to set the x-dimension for
   * @param value Value to set in m
   * @param confidence Confidence of the value in m (optional)
   */
inline void setXDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const uint8_t confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  setObjectDimension(object.object_dimension_x, (uint16_t)(value * 10), confidence * 10);
  object.object_dimension_x_is_present = true;
}

/**
   * @brief Set the y-dimension of the PerceivedObject
   *
   * Sets the y-dimension of the PerceivedObject to the given value.
   *
   * @param object PerceivedObject to set the y-dimension for
   * @param value Value to set in m
   * @param confidence Confidence of the value in m (optional)
   */
inline void setYDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const uint8_t confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  setObjectDimension(object.object_dimension_y, (uint16_t)(value * 10), confidence * 10);
  object.object_dimension_y_is_present = true;
}

/**
   * @brief Set the z-dimension of the PerceivedObject
   *
   * Sets the z-dimension of the PerceivedObject to the given value.
   *
   * @param object PerceivedObject to set the z-dimension for
   * @param value Value to set in m
   * @param confidence Confidence of the value in m (optional)
   */
inline void setZDimensionOfPerceivedObject(PerceivedObject& object, const double value,
                                           const uint8_t confidence = ObjectDimensionConfidence::UNAVAILABLE) {
  setObjectDimension(object.object_dimension_z, (uint16_t)(value * 10), confidence * 10);
  object.object_dimension_z_is_present = true;
}

/**
   * @brief Set the dimensions of the PerceivedObject
   *
   * Sets the dimensions of the PerceivedObject to the given dimensions.
   *
   * @param object PerceivedObject to set the dimensions for
   * @param dimensions Dimensions to set in m
   * @param x_confidence Confidence of the x-dimension in m (optional)
   * @param y_confidence Confidence of the y-dimension in m (optional)
   * @param z_confidence Confidence of the z-dimension in m (optional)
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
   * @brief initialize a PerceivedObject
   *
   * Initializes all necessary fields of a PerceivedObject.
   *
   * @param object PerceivedObject to be initialized
   * @param point Point to set the position to in m
   * @param delta_time Delta time to set in milliseconds
   */
inline void initPerceivedObject(PerceivedObject& object, const gm::Point& point, const int16_t delta_time = 0) {
  setPositionOfPerceivedObject(object, point);
  setMeasurementDeltaTimeOfPerceivedObject(object, delta_time);
}

/**
   * @brief initialize a PerceivedObject
   *
   * Initializes all necessary fields of a PerceivedObject.
   *
   * @param object PerceivedObject to be initialized
   * @param point UTM Point to set the position to in m
   * @param delta_time Delta time to set in milliseconds
   */
inline void initPerceivedObjectWithUTMPosition(CollectivePerceptionMessage& cpm, PerceivedObject& object,
                                               const gm::PointStamped& point, const int16_t delta_time = 0) {
  setUTMPositionOfPerceivedObject(cpm, object, point);
  setMeasurementDeltaTimeOfPerceivedObject(object, delta_time);
}

/**
 * @brief Initializes the Perceived Object Container within a WrappedCpmContainer.
 *
 * This function sets the container ID to PERCEIVED_OBJECT_CONTAINER and initializes
 * the number of perceived objects in the container to the specified value.
 *
 * @param container A reference to the WrappedCpmContainer to be initialized.
 * @param n_objects The number of perceived objects to initialize in the container. Default is 0.
 */
inline void initPerceivedObjectContainer(WrappedCpmContainer& container, const uint8_t n_objects = 0) {
  container.container_id.value = CpmContainerId::PERCEIVED_OBJECT_CONTAINER;
  container.container_data.choice = container.container_id;
  container.container_data.perceived_object_container.number_of_perceived_objects.value = n_objects;
}

/**
 * @brief Adds a PerceivedObject to the PerceivedObjectContainer within the given WrappedCpmContainer.
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
  if (container.container_id.value == CpmContainerId::PERCEIVED_OBJECT_CONTAINER &&
      container.container_data.choice.value == CpmContainerId::PERCEIVED_OBJECT_CONTAINER) {
    container.container_data.perceived_object_container.perceived_objects.array.push_back(perceived_object);
    container.container_data.perceived_object_container.number_of_perceived_objects.value =
        container.container_data.perceived_object_container.perceived_objects.array.size();
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
