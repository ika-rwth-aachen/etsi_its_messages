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
   * @brief Get the UTM Position defined within the BasicContainer of the CPM
   *
   * The position is transformed into UTM by using GeographicLib::UTMUPS
   * The altitude value is directly used as z-Coordinate
   *
   * @param[in] cpm CPM to get the UTM Position from
   * @param[out] zone the UTM zone (zero means UPS)
   * @param[out] northp hemisphere (true means north, false means south)
   * @return gm::PointStamped geometry_msgs::PointStamped of the given position
   */
inline gm::PointStamped getUTMPosition(const CollectivePerceptionMessage &cpm, int &zone, bool &northp) {
  return getUTMPosition(cpm.payload.management_container.reference_position, zone, northp);
}

/**
   * @brief Get the number of perceived objects in the CPM
   * 
   * @param cpm CPM to get the number of perceived objects from
   * @return int the number of perceived objects
   */
inline int getNumberOfPerceivedObjects(const etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage &cpm) {
  int number = cpm.payload.cpm_containers.value.array[0]
                   .container_data.perceived_object_container.number_of_perceived_objects.value;
  return number;
}

// Getters for the PerceivedObject

/**
   * @brief Get the PerceivedObject from the CPM
   * 
   * @param cpm CPM to get the PerceivedObject from
   * @param number_of_object the number of the object to get
   * @return PerceivedObject the PerceivedObject
   */
inline etsi_its_cpm_ts_msgs::PerceivedObject getPerceivedObject(const CollectivePerceptionMessage &cpm,
                                                                int number_of_object) {
  return cpm.payload.cpm_containers.value.array[0]
      .container_data.perceived_object_container.perceived_objects.array[number_of_object];
}

//inline gm::PoseWithCovariance getPoseWithCovarianceOfPerceivedObject(const PerceivedObject &object) {}

/**
   * @brief Get the pose of the PerceivedObject
   * 
   * @param object PerceivedObject to get the pose from
   * @return gm::pose the pose of the PerceivedObject
   */
inline gm::Pose getPoseOfPerceivedObject(const PerceivedObject &object) {
  geometry_msgs::msg::Pose pose;
  pose.position = getPositionOfPerceivedObject(object);
  pose.orientation = getOrientationOfPerceivedObject(object);
  return pose;
}

/**
   * @brief Get the position of the PerceivedObject
   * 
   * @param object PerceivedObject to get the position from
   * @return gm::Point the position of the PerceivedObject
   */
inline gm::Point getPositionOfPerceivedObject(const PerceivedObject &object) {
  geometry_msgs::msg::Point point;
  point.x = getXPositionOfPerceivedObject(object);
  point.y = getYPositionOfPerceivedObject(object);
  point.z = getZPositionOfPerceivedObject(object);
  return point;
}

/**
   * @brief Get the x-coordinate of the PerceivedObject
   * 
   * @param object PerceivedObject to get the x-coordinate from
   * @return double the x-coordinate of the PerceivedObject
   */
inline double getXPositionOfPerceivedObject(const PerceivedObject &object) {
  double altitude = object.position.z_coordinate.value.value / 100.0;
  return altitude;
}

/**
   * @brief Get the y-coordinate of the PerceivedObject
   * 
   * @param object PerceivedObject to get the y-coordinate from
   * @return double the y-coordinate of the PerceivedObject
   */
inline double getYPositionOfPerceivedObject(const PerceivedObject &object) {
  double longitude = object.position.y_coordinate.value.value / 100.0;
  return longitude;
}

/**
   * @brief Get the z-coordinate of the PerceivedObject
   * 
   * @param object PerceivedObject to get the z-coordinate from
   * @return double the z-coordinate of the PerceivedObject
   */
inline double getZPositionOfPerceivedObject(const PerceivedObject &object) {
  double latitude = object.position.x_coordinate.value.value / 100.0;
  return latitude;
}

/**
   * @brief Get the orientation of the PerceivedObject
   * 
   * @param object PerceivedObject to get the orientation from
   * @return gm::Quaternion the orientation of the PerceivedObject
   */
inline gm::Quaternion getOrientationOfPerceivedObject(const PerceivedObject &object) {
  geometry_msgs::msg::Quaternion q;

  double yaw = getYawOfObject(object);
  double pitch = getPitchOfObject(object);
  double roll = getRollOfObject(object);

  yaw = yaw * M_PI / 180;
  pitch = pitch * M_PI / 180;
  roll = roll * M_PI / 180;

  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}

/**
   * @brief Get the roll of the PerceivedObject
   * 
   * @param object PerceivedObject to get the roll from
   * @return double the roll of the PerceivedObject
   */
inline double getRollOfPerceivedObject(const PerceivedObject &object) {
  double roll = object.angles.x_angle.value.value / 100.0;
  return roll;
}

/**
   * @brief Get the pitch of the PerceivedObject
   * 
   * @param object PerceivedObject to get the pitch from
   * @return double the pitch of the PerceivedObject
   */
inline double getPitchOfPerceivedObject(const PerceivedObject &object) {
  double pitch = object.angles.y_angle.value.value / 100.0;
  return pitch;
}

/**
   * @brief Get the yaw of the PerceivedObject
   * 
   * @param object PerceivedObject to get the yaw from
   * @return double the yaw of the PerceivedObject
   */
inline double getYawOfPerceivedObject(const PerceivedObject &object) {
  double yaw = object.angles.z_angle.value.value / 100.0;
  return yaw;
}

/**
   * @brief Get the yaw rate of the PerceivedObject
   * 
   * @param object PerceivedObject to get the yaw rate from
   * @return double the yaw rate of the PerceivedObject
   */
inline double getYawRateOfPerceivedObject(const PerceivedObject &object) {}


/**
   * @brief Get the velocity of the PerceivedObject
   * 
   * @param object PerceivedObject to get the velocity from
   * @return gm::Vector3 the velocity of the PerceivedObject
   */
inline gm::Vector3 getVelocityOfPerceivedObject(const PerceivedObject &object) {
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = getXVelocityOfPerceivedObject;
  velocity.y = getYVelocityOfPerceivedObject;
  velocity.z = getZVelocityOfPerceivedObject;
  return velocity;
}

/**
   * @brief Get the x-velocity of the PerceivedObject
   * 
   * @param object PerceivedObject to get the x-velocity from
   * @return double the x-velocity of the PerceivedObject
   */
inline double getXVelocityOfPerceivedObject(const PerceivedObject &object) {

  double velocity_x = object.velocity.cartesian_velocity.x_velocity.value.value;
  return velocity_x;
}

/**
   * @brief Get the y-velocity of the PerceivedObject
   * 
   * @param object PerceivedObject to get the y-velocity from
   * @return double the y-velocity of the PerceivedObject
   */
inline double getYVelocityOfPerceivedObject(const PerceivedObject &object) {
  double velocity_y = object.velocity.cartesian_velocity.y_velocity.value.value;
  return velocity_y;
}

/**
   * @brief Get the z-velocity of the PerceivedObject
   * 
   * @param object PerceivedObject to get the z-velocity from
   * @return double the z-velocity of the PerceivedObject
   */
inline double getZVelocityOfPerceivedObject(const PerceivedObject &object) {
  double velocity_z = object.velocity.cartesian_velocity.z_velocity.value.value;
  return velocity_z;
}

/**
   * @brief Get the acceleration of the PerceivedObject
   * 
   * @param object PerceivedObject to get the acceleration from
   * @return gm::Vector3 the acceleration of the PerceivedObject
   */
inline gm::Vector3 getAccelerationOfPerceivedObject(const PerceivedObject &object) {
  geometry_msgs::msg::Vector3 acceleration;
  acceleration.x = getXAccelerationOfPerceivedObject;
  acceleration.y = getYAccelerationOfPerceivedObject;
  acceleration.z = getZAccelerationOfPerceivedObject;
  return acceleration;
}

/**
   * @brief Get the x-acceleration of the PerceivedObject
   * 
   * @param object PerceivedObject to get the x-acceleration from
   * @return double the x-acceleration of the PerceivedObject
   */
inline double getXAccelerationOfPerceivedObject(const PerceivedObject &object) {
  double acceleration_x = object.acceleration.cartesian_acceleration.x_acceleration.value.value;
  return acceleration_x;
}

/**
   * @brief Get the y-acceleration of the PerceivedObject
   * 
   * @param object PerceivedObject to get the y-acceleration from
   * @return double the y-acceleration of the PerceivedObject
   */
inline double getYAccelerationOfPerceivedObject(const PerceivedObject &object) {
  double acceleration_y = object.acceleration.cartesian_acceleration.y_acceleration.value.value;
  return acceleration_y;
}

/**
   * @brief Get the z-acceleration of the PerceivedObject
   * 
   * @param object PerceivedObject to get the z-acceleration from
   * @return double the z-acceleration of the PerceivedObject
   */
inline double getZAccelerationOfPerceivedObject(const PerceivedObject &object) {
  double acceleration_z = object.acceleration.cartesian_acceleration.z_acceleration.value.value;
  return acceleration_z;
}

/**
   * @brief Get the dimensions of the PerceivedObject
   * 
   * @param object PerceivedObject to get the dimensions from
   * @return gm::Vector3 the dimensions of the PerceivedObject
   */
inline gm::Vector3 getDimensionsOfPerceivedObject(const PerceivedObject &object) {
  geometry_msgs::msg::Vector3 dimensions;
  dimensions.x = getXDimensionOfPerceivedObject(object);
  dimensions.y = getYDimensionOfPerceivedObject(object);
  dimensions.z = getZDimensionOfPerceivedObject(object);
  return dimensions;
}

/**
   * @brief Get the x-dimension of the PerceivedObject
   * 
   * @param object PerceivedObject to get the x-dimension from
   * @return double the x-dimension of the PerceivedObject
   */
inline double getXDimensionOfPerceivedObject(const PerceivedObject &object) {
  double x_dimension = object.object_dimension_x.value.value / 10.0;
  return x_dimension;
}

/**
   * @brief Get the y-dimension of the PerceivedObject
   * 
   * @param object PerceivedObject to get the y-dimension from
   * @return double the y-dimension of the PerceivedObject
   */
inline double getYDimensionOfPerceivedObject(const PerceivedObject &object) {
  double y_dimension = object.object_dimension_y.value.value / 10.0;
  return y_dimension;
}

/**
   * @brief Get the z-dimension of the PerceivedObject
   * 
   * @param object PerceivedObject to get the z-dimension from
   * @return double the z-dimension of the PerceivedObject
   */
inline double getZDimensionOfPerceivedObject(const PerceivedObject &object) {
  double z_dimension = object.object_dimension_z.value.value / 10.0;
  return z_dimension;
}

/**
   * @brief Get the id of the PerceivedObject
   * 
   * @param object PerceivedObject to get the id from
   * @return uint16_t the id of the PerceivedObject
   */
inline uint16_t getIdOfPerceivedObject(const PerceivedObject &object) {
  int id = object.object_id.value;
  return id;
}

}  // namespace etsi_its_cpm_ts_msgs::access
