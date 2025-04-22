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
 * @file impl/cpm/cpm_ts_getters.h
 * @brief Getter functions for the ETSI ITS CPM (TS)
 */

#pragma once

#include <tuple>

namespace etsi_its_cpm_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-1-1_getters.h>

/**
   * @brief Retrieves the station ID from the given CPM.
   *
   * This function extracts the station ID from the header of the provided CPM.
   *
   * @param cpm The CPM from which to retrieve the station ID.
   * @return The station ID extracted from the header of the CPM.
   */
inline uint32_t getStationID(const CollectivePerceptionMessage &cpm) { return getStationID(cpm.header); }

/**
   * @brief Get the Reference Time object
   * 
   * @param cpm CPM to get the ReferenceTime-Value from
   * @return TimestampIts
   */
inline TimestampIts getReferenceTime(const CollectivePerceptionMessage &cpm) {
  return cpm.payload.management_container.reference_time;
}

/**
   * @brief Get the ReferenceTime-Value
   * 
   * @param cpm CPM to get the ReferenceTime-Value from 
   * @return uint64_t the ReferenceTime-Value in milliseconds
   */
inline uint64_t getReferenceTimeValue(const CollectivePerceptionMessage &cpm) { return getReferenceTime(cpm).value; }

/**
   * @brief Get the Latitude value of CPM
   * 
   * @param cpm CPM to get the Latitude value from
   * @return Latitude value in degree as decimal number
   */
inline double getLatitude(const CollectivePerceptionMessage &cpm) {
  return getLatitude(cpm.payload.management_container.reference_position.latitude);
}

/**
   * @brief Get the Longitude value of CPM
   * 
   * @param cpm CPM to get the Longitude value from
   * @return Longitude value in degree as decimal number
   */
inline double getLongitude(const CollectivePerceptionMessage &cpm) {
  return getLongitude(cpm.payload.management_container.reference_position.longitude);
}

/**
   * @brief Get the Altitude value of CPM
   * 
   * @param cpm CPM to get the Altitude value from
   * @return Altitude value in (above the reference ellipsoid surface) in meter as decimal number
   */
inline double getAltitude(const CollectivePerceptionMessage &cpm) {
  return getAltitude(cpm.payload.management_container.reference_position.altitude);
}

/**
   * @brief Get the UTM Position defined within the ManagementContainer of the CPM
   *
   * The position is transformed into UTM by using GeographicLib::UTMUPS
   * The altitude value is directly used as z-Coordinate
   *
   * @param[in] cpm CPM to get the UTM Position from
   * @param[out] zone the UTM zone (zero means UPS)
   * @param[out] northp hemisphere (true means north, false means south)
   * @return geometry_msgs::PointStamped of the given position with the UTM zone and hemisphere as frame_id
   */
inline gm::PointStamped getUTMPosition(const CollectivePerceptionMessage &cpm, int &zone, bool &northp) {
  return getUTMPosition(cpm.payload.management_container.reference_position, zone, northp);
}

/**
 * @brief Get the UTM Position defined within the ManagementContainer of the CPM
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param cpm CPM from which to extract the UTM position.
 * @return geometry_msgs::PointStamped of the given position with the UTM zone and hemisphere as frame_id
 */
inline gm::PointStamped getUTMPosition(const CollectivePerceptionMessage &cpm) {
  int zone;
  bool northp;
  return getUTMPosition(cpm, zone, northp);
}

/**
 * @brief Retrieves the container IDs from a CPM.
 *
 * This function iterates over the cpm_containers array in the given CPM
 * and extracts the container IDs into a vector of uint8_t.
 *
 * @param cpm The CPM from which to retrieve the container IDs.
 * @return A vector containing the container IDs.
 */
inline std::vector<uint8_t> getCpmContainerIds(const CollectivePerceptionMessage &cpm) {
  std::vector<uint8_t> container_ids;
  for (int i = 0; i < cpm.payload.cpm_containers.value.array.size(); i++) {
    container_ids.push_back(cpm.payload.cpm_containers.value.array[i].container_id.value);
  }
  return container_ids;
}

/**
 * Retrieves the CpmContainer with the specified container ID from the CPM.
 *
 * @param cpm The CPM from which to retrieve the CpmContainer.
 * @param container_id The ID of the CpmContainer to retrieve.
 * @return The CpmContainer with the specified container ID.
 * @throws std::invalid_argument if no CpmContainer with the specified ID is found in the CPM.
 */
inline WrappedCpmContainer getCpmContainer(const CollectivePerceptionMessage &cpm, const uint8_t container_id) {
  for (int i = 0; i < cpm.payload.cpm_containers.value.array.size(); i++) {
    if (cpm.payload.cpm_containers.value.array[i].container_id.value == container_id) {
      return cpm.payload.cpm_containers.value.array[i];
    }
  }
  throw std::invalid_argument("No Container with ID " + std::to_string(container_id) + " found in CPM");
}

/**
 * @brief Retrieves the perceived object container from a CPM.
 *
 * This function returns the perceived object container from the given CPM.
 *
 * @param cpm The CPM from which to retrieve the perceived object container.
 * @return The perceived object container.
 */
inline WrappedCpmContainer getPerceivedObjectContainer(const CollectivePerceptionMessage &cpm) {
  return getCpmContainer(cpm, WrappedCpmContainer::CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER);
}

/**
 * Retrieves the number of perceived objects from the given perceived object container.
 *
 * @param container The perceived object container to retrieve the number of perceived objects from.
 * @return The number of perceived objects.
 * @throws std::invalid_argument if the container is not a PerceivedObjectContainer.
 */
inline uint8_t getNumberOfPerceivedObjects(const WrappedCpmContainer &container) {
  if (container.container_id.value != WrappedCpmContainer::CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER) {
    throw std::invalid_argument("Container is not a PerceivedObjectContainer");
  }
  uint8_t number = container.container_data_perceived_object_container.number_of_perceived_objects.value;
  return number;
}

/**
 * Retrieves the number of perceived objects from the given CPM.
 *
 * @param cpm The CPM from which to retrieve the number of perceived objects.
 * @return The number of perceived objects.
 */
inline uint8_t getNumberOfPerceivedObjects(const CollectivePerceptionMessage &cpm) {
  return getNumberOfPerceivedObjects(getPerceivedObjectContainer(cpm));
}

//////////////////////////////////////////////////////////////////////////
/////////////////// getters for the PerceivedObject //////////////////////
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Retrieves the PerceivedObject at the specified index from the given WrappedCpmContainer.
 *
 * @param container The WrappedCpmContainer from which to retrieve the PerceivedObject (should be a PerceivedObjectContainer).
 * @param i The index of the PerceivedObject to retrieve.
 * @return The PerceivedObject at the specified index.
 * @throws std::invalid_argument if the index is out of range.
 */
inline PerceivedObject getPerceivedObject(const WrappedCpmContainer &container, const uint8_t i) {
  if (i >= getNumberOfPerceivedObjects(container)) {
    throw std::invalid_argument("Index out of range");
  }
  return container.container_data_perceived_object_container.perceived_objects.array[i];
}

/**
 * @brief Retrieves the ID of a perceived object.
 *
 * This function takes a PerceivedObject as input and returns the ID of the object.
 *
 * @param object The PerceivedObject for which to retrieve the ID.
 * @return The ID of the perceived object.
 */
inline uint16_t getIdOfPerceivedObject(const PerceivedObject &object) { return object.object_id.value; }

/**
 * @brief Retrieves the Cartesian coordinate value from a CartesianCoordinateWithConfidence object.
 *
 * @param coordinate The CartesianCoordinateWithConfidence object from which to retrieve the value.
 * @return The Cartesian coordinate value in centimeters.
 */
inline int32_t getCartesianCoordinate(const CartesianCoordinateWithConfidence &coordinate) {
  return coordinate.value.value;
}

/**
 * @brief Retrieves the confidence value from a CartesianCoordinateWithConfidence object.
 *
 * @param coordinate The CartesianCoordinateWithConfidence object from which to retrieve the confidence value.
 * @return The confidence value of the CartesianCoordinateWithConfidence object in centimeters.
 */
inline uint16_t getCartesianCoordinateConfidence(const CartesianCoordinateWithConfidence &coordinate) {
  return coordinate.confidence.value;
}

/**
 * Returns the position of a perceived object.
 *
 * @param object The perceived object.
 * @return The position of the perceived object as a gm::Point (all values in meters).
 */
inline gm::Point getPositionOfPerceivedObject(const PerceivedObject &object) {
  gm::Point point;
  point.x = double(getCartesianCoordinate(object.position.x_coordinate)) / 100.0;
  point.y = double(getCartesianCoordinate(object.position.y_coordinate)) / 100.0;
  if (object.position.z_coordinate_is_present) {
    point.z = double(getCartesianCoordinate(object.position.z_coordinate)) / 100.0;
  }
  return point;
}

/**
 * @brief Get the Position Confidences Of Perceived Object
 * 
 * @param object The PerceivedObject to get the position confidences from
 * @return std::tuple<double, double, double> x,y and z standard deviations of the positions in meters. 
 *         The z standard deviation is infinity if the z coordinate is not present.
 */
inline std::tuple<double, double, double> getPositionConfidenceOfPerceivedObject(const PerceivedObject &object) {
  double x_confidence = double(getCartesianCoordinateConfidence(object.position.x_coordinate)) / 100.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
  double y_confidence = double(getCartesianCoordinateConfidence(object.position.y_coordinate)) / 100.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
  double z_confidence = object.position.z_coordinate_is_present
                            ? double(getCartesianCoordinateConfidence(object.position.z_coordinate)) / 100.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR
                            : std::numeric_limits<double>::infinity();
  return std::make_tuple(x_confidence, y_confidence, z_confidence);
}

/**
 * @brief Get the Cartesian angle of the PerceivedObject
 * 
 * @param object PerceivedObject to get the Cartesian angle from
 * @return unit16_t Cartesian angle of the PerceivedObject in 0,1 degrees
 */
inline uint16_t getCartesianAngle(const CartesianAngle &angle) { return angle.value.value; }

/**
 * @brief Get the confidence of the Cartesian angle
 * 
 * @param angle CartesianAngle to get the confidence from
 * @return uint8_t confidence of the Cartesian angle in 0,1 degrees
 */
inline uint8_t getCartesianAngleConfidence(const CartesianAngle &angle) { return angle.confidence.value; }

/**
 * @brief Calculates the orientation of a perceived object.
 *
 * This function calculates the orientation of a perceived object based on the angles provided in the `PerceivedObject` structure.
 * The angles are converted to radians and used to set the roll, pitch, and yaw values of a `tf2::Quaternion` object.
 * The resulting quaternion is then converted to a `gm::Quaternion` message and returned.
 *
 * @param object The `PerceivedObject` structure containing the angles of the perceived object.
 * @return gm::Quaternion The orientation of the perceived object in radians from -pi to pi.
 */
inline gm::Quaternion getOrientationOfPerceivedObject(const PerceivedObject &object) {
  if (!object.angles_is_present) throw std::invalid_argument("No angles present in PerceivedObject");
  tf2::Quaternion q;
  double roll{0}, pitch{0}, yaw{0};

  if (object.angles.x_angle_is_present) {
    roll = (((double(getCartesianAngle(object.angles.x_angle)) / 10.0) ) / 180.0) *
           M_PI;
  }
  if (object.angles.y_angle_is_present) {
    pitch = (((double(getCartesianAngle(object.angles.y_angle)) / 10.0) ) / 180.0) *
            M_PI;
  }
  yaw = (((double(getCartesianAngle(object.angles.z_angle)) / 10.0)) / 180.0) *
        M_PI;
  // This wraps from -pi ti pi because setRPY only uses cos and sin of the angles
  q.setRPY(roll, pitch, yaw);

  return tf2::toMsg(q);
}

/**
 * @brief Get the yaw of the PerceivedObject
 * 
 * @param object PerceivedObject to get the yaw from
 * @return double yaw of the PerceivedObject in radians from -pi to pi
 */
inline double getYawOfPerceivedObject(const PerceivedObject &object) {
  gm::Quaternion orientation = getOrientationOfPerceivedObject(object);
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

/**
 * @brief Get the Yaw Confidence Of Perceived Object object
 * 
 * @param object PerceivedObject to get the yaw confidence from
 * @return double The standard deviation of the yaw angle in radians
 * @throws std::invalid_argument If the angles are not present in the object.
 */
inline double getYawConfidenceOfPerceivedObject(const PerceivedObject &object) {
  if(!object.angles_is_present) throw std::invalid_argument("No angles present in PerceivedObject");
  double yaw_confidence = object.angles.z_angle.confidence.value / 10.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
  yaw_confidence *= M_PI / 180.0;  // convert to radians
  return yaw_confidence;
}

/**
 * @brief Get the pose of the PerceivedObject
 * 
 * @param object PerceivedObject to get the pose from
 * @return gm::Pose pose of the PerceivedObject (position in m, orientation in rad)
 */
inline gm::Pose getPoseOfPerceivedObject(const PerceivedObject &object) {
  gm::Pose pose;
  pose.position = getPositionOfPerceivedObject(object);
  pose.orientation = getOrientationOfPerceivedObject(object);
  return pose;
}

/**
 * @brief Get the yaw rate of the PerceivedObject
 * 
 * @param object PerceivedObject to get the yaw rate from
 * @return double yaw rate of the PerceivedObject in rad/s
 * @throws std::invalid_argument If the yaw rate is not present in the object.
 */
inline double getYawRateOfPerceivedObject(const PerceivedObject &object) {
  if (!object.z_angular_velocity_is_present) throw std::invalid_argument("No yaw rate present in PerceivedObject");
  return object.z_angular_velocity.value.value * M_PI / 180.0;  // convert to rad/s
}

/**
 * @brief Get the Yaw Rate Confidence Of Perceived Object
 * 
 * @param object The PerceivedObject to get the yaw rate confidence from
 * @return double The standard deviation of the yaw rate in rad/s
 *                As defined in the TS, this is rounded up to the next possible value
 * @throws std::invalid_argument If the yaw rate is not present in the object.
 * 
 */
inline double getYawRateConfidenceOfPerceivedObject(const PerceivedObject &object) {
  if (!object.z_angular_velocity_is_present) throw std::invalid_argument("No yaw rate present in PerceivedObject");
  auto val = object.z_angular_velocity.confidence.value;
  static const std::map<uint8_t, double> confidence_map = {
      {AngularSpeedConfidence::UNAVAILABLE, 0.0},
      {AngularSpeedConfidence::DEG_SEC_01, 1.0},
      {AngularSpeedConfidence::DEG_SEC_02, 2.0},
      {AngularSpeedConfidence::DEG_SEC_05, 5.0},
      {AngularSpeedConfidence::DEG_SEC_10, 10.0},
      {AngularSpeedConfidence::DEG_SEC_20, 20.0},
      {AngularSpeedConfidence::DEG_SEC_50, 50.0},
      {AngularSpeedConfidence::OUT_OF_RANGE, std::numeric_limits<double>::infinity()},
  };
  return confidence_map.at(val) * M_PI / 180.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;  // convert to rad/s
}

/**
 * @brief Get the velocity component of the PerceivedObject
 * 
 * @param velocity VelocityComponent to get the value from
 * @return double value of the velocity component in m/s
 */
inline double getVelocityComponent(const VelocityComponent &velocity) { return double(velocity.value.value) / 100.0; }

/**
 * @brief Get the confidence of the velocity component
 * 
 * @param velocity VelocityComponent to get the confidence from
 * @return double value of the confidence of the velocity component in m/s
 */
inline double getVelocityComponentConfidence(const VelocityComponent &velocity) {
  return double(velocity.confidence.value) / 100.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
}

/**
 * @brief Get the Cartesian velocity of the PerceivedObject
 * 
 * @param object PerceivedObject to get the Cartesian velocity from
 * @return gm::Vector3 Cartesian velocity of the PerceivedObject in m/s (local coordinate system)
 * @throws std::invalid_argument If the velocity is no cartesian velocity.
 */
inline gm::Vector3 getCartesianVelocityOfPerceivedObject(const PerceivedObject &object) {
  if (!object.velocity_is_present) throw std::invalid_argument("No velocity present in PerceivedObject");
  if (object.velocity.choice != Velocity3dWithConfidence::CHOICE_CARTESIAN_VELOCITY) {
    throw std::invalid_argument("Velocity is not Cartesian");
  }
  gm::Vector3 velocity;
  velocity.x = getVelocityComponent(object.velocity.cartesian_velocity.x_velocity);
  velocity.y = getVelocityComponent(object.velocity.cartesian_velocity.y_velocity);
  if (object.velocity.cartesian_velocity.z_velocity_is_present) {
    velocity.z = getVelocityComponent(object.velocity.cartesian_velocity.z_velocity);
  }
  return velocity;
}

/**
 * @brief Get the Cartesian Velocity Confidence Of Perceived Object object
 * 
 * @param object PerceivedObject to get the Cartesian velocity from
 * @return std::tuple<double, double, double> the x,y and z standard deviations of the velocity in m/s
 *         The z standard deviation is infinity if the z velocity is not present.
 * @throws std::invalid_argument If the velocity is no cartesian velocity.
 */
inline std::tuple<double, double, double> getCartesianVelocityConfidenceOfPerceivedObject(const PerceivedObject &object) {
  if (!object.velocity_is_present) throw std::invalid_argument("No velocity present in PerceivedObject");
  if (object.velocity.choice != Velocity3dWithConfidence::CHOICE_CARTESIAN_VELOCITY) {
    throw std::invalid_argument("Velocity is not Cartesian");
  }
  double x_confidence = getVelocityComponentConfidence(object.velocity.cartesian_velocity.x_velocity);
  double y_confidence = getVelocityComponentConfidence(object.velocity.cartesian_velocity.y_velocity);
  double z_confidence = object.velocity.cartesian_velocity.z_velocity_is_present
                            ? getVelocityComponentConfidence(object.velocity.cartesian_velocity.z_velocity)
                            : std::numeric_limits<double>::infinity();
  return std::make_tuple(x_confidence, y_confidence, z_confidence);
}

/**
 * @brief Get the acceleration component of the PerceivedObject
 * 
 * @param acceleration AccelerationComponent to get the value from
 * @return double value of the acceleration component in m/s^2
 */
inline double getAccelerationComponent(const AccelerationComponent &acceleration) {
  return double(acceleration.value.value) / 10.0;
}

/**
 * @brief Get the confidence of the acceleration component
 * 
 * @param acceleration AccelerationComponent to get the confidence from
 * @return double value of the confidence of the acceleration component in m/s^2
 */
inline double getAccelerationComponentConfidence(const AccelerationComponent &acceleration) {
  return double(acceleration.confidence.value) / 10.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
}

/**
 * @brief Get the Cartesian acceleration of the PerceivedObject
 * 
 * @param object PerceivedObject to get the Cartesian acceleration from
 * @return gm::Vector3 Cartesian acceleration of the PerceivedObject in m/s^2 (local coordinate system)
 * @throws std::invalid_argument If the acceleration is no cartesian acceleration.
 */
inline gm::Vector3 getCartesianAccelerationOfPerceivedObject(const PerceivedObject &object) {
  if (!object.acceleration_is_present) throw std::invalid_argument("No acceleration present in PerceivedObject");
  if (object.acceleration.choice != Acceleration3dWithConfidence::CHOICE_CARTESIAN_ACCELERATION) {
    throw std::invalid_argument("Acceleration is not Cartesian");
  }
  gm::Vector3 acceleration;
  acceleration.x = getAccelerationComponent(object.acceleration.cartesian_acceleration.x_acceleration);
  acceleration.y = getAccelerationComponent(object.acceleration.cartesian_acceleration.y_acceleration);
  if (object.acceleration.cartesian_acceleration.z_acceleration_is_present) {
    acceleration.z = getAccelerationComponent(object.acceleration.cartesian_acceleration.z_acceleration);
  }
  return acceleration;
}

/**
 * @brief Get the Cartesian Acceleration Confidence Of Perceived Object 
 * 
 * @param object PerceivedObject to get the Cartesian acceleration from
 * @return std::tuple<double, double, double> the x,y and z standard deviations of the acceleration in m/s^2
 *        The z standard deviation is infinity if the z acceleration is not present.
 * @throws std::invalid_argument If the acceleration is no cartesian acceleration.
 */
inline std::tuple<double, double, double> getCartesianAccelerationConfidenceOfPerceivedObject(const PerceivedObject &object) {
  if (!object.acceleration_is_present) throw std::invalid_argument("No acceleration present in PerceivedObject");
  if (object.acceleration.choice != Acceleration3dWithConfidence::CHOICE_CARTESIAN_ACCELERATION) {
    throw std::invalid_argument("Acceleration is not Cartesian");
  }
  double x_confidence = getAccelerationComponentConfidence(object.acceleration.cartesian_acceleration.x_acceleration);
  double y_confidence = getAccelerationComponentConfidence(object.acceleration.cartesian_acceleration.y_acceleration);
  double z_confidence = object.acceleration.cartesian_acceleration.z_acceleration_is_present
                            ? getAccelerationComponentConfidence(object.acceleration.cartesian_acceleration.z_acceleration)
                            : std::numeric_limits<double>::infinity();
  return std::make_tuple(x_confidence, y_confidence, z_confidence);
}

/**
 * @brief Gets the x-dimension of a perceived object.
 *
 * This function extracts the x-dimension from a given PerceivedObject.
 * If the x-dimension is not present in the object, it throws an 
 * std::invalid_argument exception.
 *
 * @param object The PerceivedObject from which to retrieve the x-dimension.
 * @return The x-dimension of the perceived object in decimeters.
 * @throws std::invalid_argument if the x-dimension is not present in the PerceivedObject.
 */
inline uint16_t getXDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_x_is_present) throw std::invalid_argument("No x-dimension present in PerceivedObject");
  return object.object_dimension_x.value.value;
}

/**
 * @brief Gets the confidence of the x-dimension of a perceived object.
 * 
 * @param object  The PerceivedObject from which to retrieve the x-dimension confidence.
 * @return uint8_t The confidence of the x-dimension of the perceived object in decimeters.
 * @throws std::invalid_argument if the x-dimension is not present in the PerceivedObject.
 */
inline uint8_t getXDimensionConfidenceOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_x_is_present) throw std::invalid_argument("No x-dimension present in PerceivedObject");
  return object.object_dimension_x.confidence.value;
}

/**
 * @brief Retrieves the y-dimension of a perceived object.
 *
 * This function extracts the y-dimension from a given PerceivedObject.
 * If the y-dimension is not present in the object, it throws an 
 * std::invalid_argument exception.
 *
 * @param object The PerceivedObject from which to retrieve the y-dimension.
 * @return uint16_t y-dimension of the perceived object in in decimeters.
 * @throws std::invalid_argument if the y-dimension is not present in the PerceivedObject.
 */
inline uint16_t getYDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_y_is_present) throw std::invalid_argument("No y-dimension present in PerceivedObject");
  return object.object_dimension_y.value.value;
}

/**
 * @brief Gets the confidence of the y-dimension of a perceived object.
 * 
 * @param object  The PerceivedObject from which to retrieve the y-dimension confidence.
 * @return uint8_t The confidence of the y-dimension of the perceived object in decimeters.
 * @throws std::invalid_argument if the y-dimension is not present in the PerceivedObject.
 */
inline uint8_t getYDimensionConfidenceOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_y_is_present) throw std::invalid_argument("No y-dimension present in PerceivedObject");
  return object.object_dimension_y.confidence.value;
}

/**
 * @brief Retrieves the z-dimension of a perceived object.
 *
 * This function extracts the z-dimension from a given PerceivedObject.
 * If the z-dimension is not present in the object, it throws an 
 * std::invalid_argument exception.
 *
 * @param object The PerceivedObject from which to retrieve the z-dimension.
 * @return uint16_t z-dimension of the perceived object in decimeters.
 * @throws std::invalid_argument If the z-dimension is not present in the object.
 */
inline uint16_t getZDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_z_is_present) throw std::invalid_argument("No z-dimension present in PerceivedObject");
  return object.object_dimension_z.value.value;
}

/**
 * @brief Gets the confidence of the z-dimension of a perceived object.
 * 
 * @param object  The PerceivedObject from which to retrieve the z-dimension confidence.
 * @return uint8_t The confidence of the z-dimension of the perceived object in decimeters.
 * @throws std::invalid_argument if the z-dimension is not present in the PerceivedObject.
 */
inline uint8_t getZDimensionConfidenceOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_z_is_present) throw std::invalid_argument("No z-dimension present in PerceivedObject");
  return object.object_dimension_z.confidence.value;
}

/**
 * @brief Retrieves the dimensions of a perceived object.
 *
 * This function extracts the dimensions of a perceived object from the given PerceivedObject.
 * The dimensions are returned as a `gm::Vector3` object with the x, y, and z dimensions in meters.
 * 
 * @param object The `PerceivedObject` for which to calculate the dimensions.
 * @return The dimensions of the perceived object as a `gm::Vector3` object in meters.
 */
inline gm::Vector3 getDimensionsOfPerceivedObject(const PerceivedObject &object) {
  gm::Vector3 dimensions;
  dimensions.x = double(getXDimensionOfPerceivedObject(object)) / 10.0;
  dimensions.y = double(getYDimensionOfPerceivedObject(object)) / 10.0;
  dimensions.z = double(getZDimensionOfPerceivedObject(object)) / 10.0;
  return dimensions;
}

/**
 * @brief Get the Dimensions Confidence Of Perceived Object 
 * 
 * @param object The PerceivedObject to get the dimensions confidence from
 * @return std::tuple<double, double, double> the x,y and z standard deviations of the dimensions in meters
 */
inline std::tuple<double, double, double> getDimensionsConfidenceOfPerceivedObject(const PerceivedObject &object) {
  double conf_x = double(getXDimensionConfidenceOfPerceivedObject(object)) / 10.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
  double conf_y = double(getYDimensionConfidenceOfPerceivedObject(object)) / 10.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
  double conf_z = double(getZDimensionConfidenceOfPerceivedObject(object)) / 10.0 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR;
  return {conf_x, conf_y, conf_z};
}

/**
 * @brief Calculates the UTM position of a perceived object based on the CPMs referencep position.
 *
 * This function takes a CPM and a PerceivedObject as input parameters and calculates the UTM position of the object.
 * The UTM position is calculated by adding the relative position of the object to the UTM position of the reference point in the CPM.
 *
 * @param cpm The Collective Perception Message (CPM) containing the reference position.
 * @param object The PerceivedObject for which the UTM position needs to be calculated.
 * @return The UTM position of the perceived object.
 */
inline gm::PointStamped getUTMPositionOfPerceivedObject(const CollectivePerceptionMessage &cpm,
                                                        const PerceivedObject &object) {
  gm::PointStamped utm_position;
  gm::PointStamped reference_position = getUTMPosition(cpm);
  gm::Point relative_position = getPositionOfPerceivedObject(object);

  utm_position.header.frame_id = reference_position.header.frame_id;
  utm_position.point.x = reference_position.point.x + relative_position.x;
  utm_position.point.y = reference_position.point.y + relative_position.y;
  utm_position.point.z = reference_position.point.z + relative_position.z;

  return utm_position;
}

/**
 * @brief Get the confidence ellipse of the reference position as Covariance matrix
 * 
 * The covariance matrix will have the entries cov_xx, cov_xy, cov_yx, cov_yy
 * where x is WGS84 North and y is East
 * 
 * @param cpm The CPM message to get the reference position from
 * @return const std::array<double, 4> the covariance matrix, as specified above
 */
inline const std::array<double, 4> getWGSRefPosConfidence(const CollectivePerceptionMessage &cpm) {
  return getWGSPosConfidenceEllipse(cpm.payload.management_container.reference_position.position_confidence_ellipse);
}

/**
 * @brief Get the sensorId of a SensorInformation object.
 * 
 * @param sensor_information The SensorInformation to get the sensorId from
 * @return uint8_t The sensorId of a SensorInformation object
 */
inline uint8_t getSensorID(const SensorInformation &sensor_information) {
  return sensor_information.sensor_id.value;
}

/**
 * @brief Get the sensorType of a SensorInformation object.
 * 
 * @param sensor_information The SensorInformation to get the sensorType from
 * @return uint8_t The sensorType of a SensorInformation object
 */
inline uint8_t getSensorType(const SensorInformation &sensor_information) {
  return sensor_information.sensor_type.value;
}

}  // namespace etsi_its_cpm_ts_msgs::access
