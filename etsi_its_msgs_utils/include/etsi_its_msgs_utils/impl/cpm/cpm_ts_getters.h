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
inline uint32_t getStationID(const CollectivePerceptionMessage &cpm) {
  return getStationID(cpm.header);
}

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
   * @return uint64_t the ReferenceTime-Value
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
   * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
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

inline std::vector<uint8_t> getCpmContainerIds(const CollectivePerceptionMessage &cpm) {
  std::vector<uint8_t> container_ids;
  for (int i = 0; i < cpm.payload.cpm_containers.value.array.size(); i++) {
    container_ids.push_back(cpm.payload.cpm_containers.value.array[i].container_id.value);
  }
  return container_ids;
}

inline WrappedCpmContainer getCpmContainer(const CollectivePerceptionMessage &cpm, const uint8_t container_id){
  for (int i = 0; i < cpm.payload.cpm_containers.value.array.size(); i++){
    if (cpm.payload.cpm_containers.value.array[i].container_id.value == container_id){
      return cpm.payload.cpm_containers.value.array[i];
    }
  }
  throw std::invalid_argument("No Container with ID " + std::to_string(container_id) + " found in CPM");
}

inline WrappedCpmContainer getPerceivedObjectContainer(const CollectivePerceptionMessage &cpm){
  return getCpmContainer(cpm, CpmContainerId::PERCEIVED_OBJECT_CONTAINER);
}

inline uint8_t getNumberOfPerceivedObjects(const WrappedCpmContainer &container) {
  if (container.container_id.value != CpmContainerId::PERCEIVED_OBJECT_CONTAINER) {
    throw std::invalid_argument("Container is not a PerceivedObjectContainer");
  }
  uint8_t number = container.container_data.perceived_object_container.number_of_perceived_objects.value;
  return number;
}

inline uint8_t getNumberOfPerceivedObjects(const CollectivePerceptionMessage &cpm) {
  return getNumberOfPerceivedObjects(getPerceivedObjectContainer(cpm));
}

// getters for the PerceivedObject

inline PerceivedObject getPerceivedObject(const WrappedCpmContainer &container, const uint8_t i) {
  if (i >= getNumberOfPerceivedObjects(container)) {
    throw std::invalid_argument("Index out of range");
  }
  return container.container_data.perceived_object_container.perceived_objects.array[i];
}

inline uint16_t getIdOfPerceivedObject(const PerceivedObject &object) {
  return object.object_id.value;
}

inline int32_t getCartesianCoordinate(const CartesianCoordinateWithConfidence &coordinate) {
  return coordinate.value.value / 100;
}

inline uint16_t getCartesianCoordinateConfidence(const CartesianCoordinateWithConfidence &coordinate) {
  return coordinate.confidence.value / 100;
}

inline gm::Point getPositionOfPerceivedObject(const PerceivedObject &object) {
  gm::Point point;
  point.x = getCartesianCoordinate(object.position.x_coordinate);
  point.y = getCartesianCoordinate(object.position.y_coordinate);
  if (object.position.z_coordinate_is_present) {
    point.z = getCartesianCoordinate(object.position.z_coordinate);
  }
  return point;
}

inline uint16_t getCartesianAngle(const CartesianAngle &angle) {
  return angle.value.value / 10;
}

inline uint8_t getCartesianAngleConfidence(const CartesianAngle &angle) {
  return angle.confidence.value / 10;
}

inline gm::Quaternion getOrientationOfPerceivedObject(const PerceivedObject &object) {
  if (!object.angles_is_present) throw std::invalid_argument("No angles present in PerceivedObject");
  tf2::Quaternion q;
  double roll{0}, pitch{0}, yaw{0};

  if (object.angles.x_angle_is_present) {

    roll = ((static_cast<double>(getCartesianAngle(object.angles.x_angle)) - 180) / 180) * M_PI;
  }
  if (object.angles.y_angle_is_present) {
    pitch = ((static_cast<double>(getCartesianAngle(object.angles.y_angle)) - 180) / 180) * M_PI;
  }
  yaw = ((static_cast<double>(getCartesianAngle(object.angles.z_angle)) - 180) / 180) * M_PI;
  q.setRPY(roll, pitch, yaw);
  
  return tf2::toMsg(q);
}

inline gm::Pose getPoseOfPerceivedObject(const PerceivedObject &object) {
  gm::Pose pose;
  pose.position = getPositionOfPerceivedObject(object);
  pose.orientation = getOrientationOfPerceivedObject(object);
  return pose;
}
//inline gm::PoseWithCovariance getPoseWithCovarianceOfPerceivedObject(const PerceivedObject &object) {}

inline int16_t getYawRateOfPerceivedObject(const PerceivedObject &object) {
  if (!object.z_angular_velocity_is_present) throw std::invalid_argument("No yaw rate present in PerceivedObject");
   return object.z_angular_velocity.value.value;
}

inline int16_t getVelocityComponent(const VelocityComponent &velocity) {
  return velocity.value.value / 100;
}

inline uint8_t getVelocityComponentConfidence(const VelocityComponent &velocity) {
  return velocity.confidence.value / 100;
}

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

inline int16_t getAccelerationComponent(const AccelerationComponent &acceleration) {
  return acceleration.value.value / 10;
}

inline uint8_t getAccelerationComponentConfidence(const AccelerationComponent &acceleration) {
  return acceleration.confidence.value / 10;
}

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

inline uint16_t getXDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_x_is_present) throw std::invalid_argument("No x-dimension present in PerceivedObject");
  return object.object_dimension_x.value.value / 10;
}

inline uint16_t getYDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_y_is_present) throw std::invalid_argument("No y-dimension present in PerceivedObject");
  return object.object_dimension_y.value.value / 10;
}

inline uint16_t getZDimensionOfPerceivedObject(const PerceivedObject &object) {
  if (!object.object_dimension_z_is_present) throw std::invalid_argument("No z-dimension present in PerceivedObject");
  return object.object_dimension_z.value.value / 10;
}

inline gm::Vector3 getDimensionsOfPerceivedObject(const PerceivedObject &object) {
  gm::Vector3 dimensions;
  dimensions.x = getXDimensionOfPerceivedObject(object);
  dimensions.y = getYDimensionOfPerceivedObject(object);
  dimensions.z = getZDimensionOfPerceivedObject(object);
  return dimensions;
}

inline gm::PointStamped getUTMPositionOfPerceivedObject(const CollectivePerceptionMessage &cpm, const PerceivedObject &object) {
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
