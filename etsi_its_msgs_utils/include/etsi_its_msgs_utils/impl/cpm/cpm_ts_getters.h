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
 * @file impl/cpm/cpm_ts_getters.h
 * @brief Getter functions for the ETSI ITS CPM (TS)
 */

#pragma once

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
   * @return Latitude value in 10^-7 degree
   */
inline double getLatitude(const CollectivePerceptionMessage &cpm) {
  return getLatitude(cpm.payload.management_container.reference_position.latitude);
}

/**
   * @brief Get the Longitude value of CPM
   * 
   * @param cpm CPM to get the Longitude value from
   * @return Longitude value in 10^-7 degree
   */
inline double getLongitude(const CollectivePerceptionMessage &cpm) {
  return getLongitude(cpm.payload.management_container.reference_position.longitude);
}

/**
   * @brief Get the Altitude value of CPM
   * 
   * @param cpm CPM to get the Altitude value from
   * @return Altitude value in centimeters
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
 * @return geometry_msgs::PointStamped of the given position with the UTM position, int UTM zone and bool hemisphere
 */
inline gm::PointStamped getUTMPosition(const CollectivePerceptionMessage &cpm) {
  int zone;
  bool northp;
  return getUTMPosition(cpm, zone, northp);
}

/**
   * @brief Get the IDs of CPM-Containers in the CPM
   * 
   * @param cpm CPM to get the number of CPM-Containers from
   * @return uint8_t IDs of CPM-Containers
   */
inline std::vector<uint8_t> getCpmContainerIds(const CollectivePerceptionMessage &cpm) {
  std::vector<uint8_t> container_ids;
  for (int i = 0; i < cpm.payload.cpm_containers.value.array.size(); i++) {
    container_ids.push_back(cpm.payload.cpm_containers.value.array[i].container_id.value);
  }
  return container_ids;
}

/**
   * @brief Get the CPM-Container with the given ID
   * 
   * @param cpm CPM to get the CPM-Container from
   * @param container_id ID of the CPM-Container
   * @return WrappedCpmContainer CPM-Container with the given ID
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
   * @brief Get the Perceived Object Container from the CPM
   * 
   * @param cpm CPM to get the CPM-Container from
   * @return WrappedCpmContainer Perceived Object Container from the CPM
   */
inline WrappedCpmContainer getPerceivedObjectContainer(const CollectivePerceptionMessage &cpm) {
  return getCpmContainer(cpm, CpmContainerId::PERCEIVED_OBJECT_CONTAINER);
}

/**
   * @brief Get the number of perceived objects in the PerceivedObjectContainer
   * 
   * @param container PerceivedObjectContainer to get the number of perceived objects from
   * @return uint8_t number of perceived objects
   */
inline uint8_t getNumberOfPerceivedObjects(const WrappedCpmContainer &container) {
  if (container.container_id.value != CpmContainerId::PERCEIVED_OBJECT_CONTAINER) {
    throw std::invalid_argument("Container is not a PerceivedObjectContainer");
  }
  uint8_t number = container.container_data.perceived_object_container.number_of_perceived_objects.value;
  return number;
}

/**
   * @brief Get the number of perceived objects in the PerceivedObjectContainer
   * 
   * @param cpm CPM to get the number of perceived objects from
   * @return uint8_t number of perceived objects
   */
inline uint8_t getNumberOfPerceivedObjects(const CollectivePerceptionMessage &cpm) {
  return getNumberOfPerceivedObjects(getPerceivedObjectContainer(cpm));
}

// getters for the PerceivedObject

/**
   * @brief Get the PerceivedObject with the given index
   * 
   * @param container PerceivedObjectContainer to get the PerceivedObject from
   * @param i index of the PerceivedObject
   * @return PerceivedObject PerceivedObject with the given index
   */
inline PerceivedObject getPerceivedObject(const WrappedCpmContainer &container, const uint8_t i) {
  if (i >= getNumberOfPerceivedObjects(container)) {
    throw std::invalid_argument("Index out of range");
  }
  return container.container_data.perceived_object_container.perceived_objects.array[i];
}

/**
   * @brief Get the ID of PerceivedObject
   * 
   * @param object PerceivedObject to get the ID from
   * @return PerceivedObject ID
   */
inline uint16_t getIdOfPerceivedObject(const PerceivedObject &object) { return object.object_id.value; }

/**
   * @brief Get the CartesianCoordinate
   * 
   * @param coordinate CartesianCoordinateWithConfidence to get the value from
   * @return value of the CartesianCoordinate in cm
   */
inline int32_t getCartesianCoordinate(const CartesianCoordinateWithConfidence &coordinate) {
  return coordinate.value.value;
}

/**
   * @brief Get the confidence of the CartesianCoordinate
   * 
   * @param coordinate CartesianCoordinateWithConfidence to get the confidence from
   * @return value of the confidence of the CartesianCoordinate in cm
   */
inline uint16_t getCartesianCoordinateConfidence(const CartesianCoordinateWithConfidence &coordinate) {
  return coordinate.confidence.value;
}

/**
   * @brief Get the position of the PerceivedObject
   * 
   * @param object PerceivedObject to get the position from
   * @return gm::Point position of the PerceivedObject in m
   */
inline gm::Point getPositionOfPerceivedObject(const PerceivedObject &object) {
  gm::Point point;
  point.x = double(getCartesianCoordinate(object.position.x_coordinate)) / 100;
  point.y = double(getCartesianCoordinate(object.position.y_coordinate)) / 100;
  if (object.position.z_coordinate_is_present) {
    point.z = double(getCartesianCoordinate(object.position.z_coordinate)) / 100;
  }
  return point;
}

/**
   * @brief Get the CartesianAngle
   * 
   * @param angle CartesianAngle to get the value from
   * @return value of the CartesianAnglein 0,1 degrees
   */
inline uint16_t getCartesianAngle(const CartesianAngle &angle) { return angle.value.value; }

/**
   * @brief Get the confidence of the CartesianAngle
   * 
   * @param angle CartesianAngle to get the confidence from
   * @return value of the confidence of the CartesianAngle in 0,1 degrees
   */
inline uint8_t getCartesianAngleConfidence(const CartesianAngle &angle) { return angle.confidence.value; }

/**
   * @brief Get the orientation of the PerceivedObject
   * 
   * @param object PerceivedObject to get the orientation from
   * @return gm::Quaternion orientation of the PerceivedObject in rad
   */
inline gm::Quaternion getOrientationOfPerceivedObject(const PerceivedObject &object) {
  if (!object.angles_is_present) throw std::invalid_argument("No angles present in PerceivedObject");
  tf2::Quaternion q;
  double roll{0}, pitch{0}, yaw{0};

  if (object.angles.x_angle_is_present) {
    roll = (((double(getCartesianAngle(object.angles.x_angle)) / 10) - 180) / 180) * M_PI;
  }
  if (object.angles.y_angle_is_present) {
    pitch = (((double(getCartesianAngle(object.angles.y_angle)) / 10) - 180) / 180) * M_PI;
  }
  yaw = (((double(getCartesianAngle(object.angles.z_angle)) / 10) - 180) / 180) * M_PI;
  q.setRPY(roll, pitch, yaw);

  return tf2::toMsg(q);
}

/**
   * @brief Get the yaw of the PerceivedObject
   * 
   * @param object PerceivedObject to get the yaw from
   * @return double yaw of the PerceivedObject in rad
   */
inline double getYawOfPerceivedObject(const PerceivedObject &object) {
  gm::Quaternion orientation = getOrientationOfPerceivedObject(object);
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

/**
   * @brief Get the pose of the PerceivedObject
   * 
   * @param object PerceivedObject to get the pose from
   * @return gm::Pose pose of the PerceivedObject
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
   * @return int16_t yaw rate of the PerceivedObject in deg/s
   */
inline int16_t getYawRateOfPerceivedObject(const PerceivedObject &object) {
  if (!object.z_angular_velocity_is_present) throw std::invalid_argument("No yaw rate present in PerceivedObject");
  return object.z_angular_velocity.value.value;
}

/**
   * @brief Get the velocity component of the PerceivedObject
   * 
   * @param velocity VelocityComponent to get the value from
   * @return double value of the velocity component in m/s
   */
inline double getVelocityComponent(const VelocityComponent &velocity) { return double(velocity.value.value) / 100; }

/**
   * @brief Get the confidence of the velocity component
   * 
   * @param velocity VelocityComponent to get the confidence from
   * @return double value of the confidence of the velocity component in m/s
   */
inline double getVelocityComponentConfidence(const VelocityComponent &velocity) {
  return double(velocity.confidence.value) / 100;
}

/**
   * @brief Get the Cartesian velocity of the PerceivedObject
   * 
   * @param object PerceivedObject to get the Cartesian velocity from
   * @return gm::Vector3 Cartesian velocity of the PerceivedObject in m/s
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
   * @brief Get the acceleration component of the PerceivedObject
   * 
   * @param acceleration AccelerationComponent to get the value from
   * @return double value of the acceleration component in m/s^2
   */
inline double getAccelerationComponent(const AccelerationComponent &acceleration) {
  return double(acceleration.value.value) / 10;
}

/**
   * @brief Get the confidence of the acceleration component
   * 
   * @param acceleration AccelerationComponent to get the confidence from
   * @return double value of the confidence of the acceleration component in m/s^2
   */
inline double getAccelerationComponentConfidence(const AccelerationComponent &acceleration) {
  return double(acceleration.confidence.value) / 10;
}

/**
   * @brief Get the Cartesian acceleration of the PerceivedObject
   * 
   * @param object PerceivedObject to get the Cartesian acceleration from
   * @return gm::Vector3 Cartesian acceleration of the PerceivedObject in m/s^2
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
   * @brief Get the x-dimensions of the PerceivedObject
   * 
   * @param object PerceivedObject to get the dimensions from
   * @return uint16_t x-dimensions of the PerceivedObject in dm
   * @throws std::invalid_argument If the Y dimension is not present in the object.
   */
inline uint16_t getXDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_x_is_present) throw std::invalid_argument("No x-dimension present in PerceivedObject");
  return object.object_dimension_x.value.value;
}

/**
 * @brief Retrieves the Y dimension of a perceived object.
 *
 * This function extracts the Y dimension from a given PerceivedObject.
 * If the Y dimension is not present in the object, it throws an 
 * std::invalid_argument exception.
 *
 * @param object The PerceivedObject from which to retrieve the Y dimension.
 * @return uint16_t Y dimension of the perceived object in dm.
 * @throws std::invalid_argument If the Y dimension is not present in the object.
 */
inline uint16_t getYDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_y_is_present) throw std::invalid_argument("No y-dimension present in PerceivedObject");
  return object.object_dimension_y.value.value;
}

/**
 * @brief Retrieves the Z dimension of a perceived object.
 *
 * This function extracts the Z dimension from a given PerceivedObject.
 * If the Z dimension is not present in the object, it throws an 
 * std::invalid_argument exception.
 *
 * @param object The PerceivedObject from which to retrieve the Z dimension.
 * @return uint16_t Z dimension of the perceived object in dm.
 * @throws std::invalid_argument If the Z dimension is not present in the object.
 */
inline uint16_t getZDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_z_is_present) throw std::invalid_argument("No z-dimension present in PerceivedObject");
  return object.object_dimension_z.value.value;
}

/**
   * @brief Get the dimensions of the PerceivedObject
   * 
   * @param object PerceivedObject to get the dimensions from
   * @return gm::Vector3 dimensions of the PerceivedObject in m
   */
inline gm::Vector3 getDimensionsOfPerceivedObject(const PerceivedObject &object) {
  gm::Vector3 dimensions;
  dimensions.x = double(getXDimensionOfPerceivedObject(object)) / 10;
  dimensions.y = double(getYDimensionOfPerceivedObject(object)) / 10;
  dimensions.z = double(getZDimensionOfPerceivedObject(object)) / 10;
  return dimensions;
}

/**
   * @brief Get the position of the PerceivedObject in UTM
   * 
   * @param cpm CPM to get the ReferencePosition from
   * @param object PerceivedObject to get the position from
   * @return gm::PointStamped position of the PerceivedObject in m
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

}  // namespace etsi_its_cpm_ts_msgs::access
