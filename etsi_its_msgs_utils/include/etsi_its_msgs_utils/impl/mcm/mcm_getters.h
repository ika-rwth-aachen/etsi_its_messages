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
 * @file impl/mcm/mcm_getters.h
 * @brief Getter functions for the UULM MCM (TR)
 */

#pragma once

namespace etsi_its_mcm_uulm_msgs::access {

/**
 * @brief Get the Latitude value
 *
 * @param latitude to get the Latitude value from
 * @return Latitude value in degree as decimal number
 */
inline double getLatitude(const Latitude& latitude) { return ((double)latitude.value) * 1e-7; }

/**
  * @brief Get the Longitude value
  *
  * @param longitude to get the Longitude value from
  * @return Longitude value in degree as decimal number
  */
inline double getLongitude(const Longitude& longitude) { return ((double)longitude.value) * 1e-7; }

/**
  * @brief Get the Altitude value
  *
  * @param altitude to get the Altitude value from
  * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number (0 if unavailable)
  */
inline double getAltitude(const Altitude& altitude) {
  if (altitude.altitude_value.value == AltitudeValue::UNAVAILABLE) {
    return 0.0;
  }

  return ((double)altitude.altitude_value.value) * 1e-2;
}

/**
 * @brief Get the UTM Position defined by the given ReferencePosition along with the grid-convergence angle
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] reference_position ReferencePosition or ReferencePositionWithConfidence to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @param[out] conv_angle grid-convergence angle in degree
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
template <typename T>
inline gm::PointStamped getUTMPosition(const T& reference_position, int& zone, bool& northp, double& conv_angle) {
  gm::PointStamped utm_point;
  double latitude = getLatitude(reference_position.latitude);
  double longitude = getLongitude(reference_position.longitude);
  utm_point.point.z = getAltitude(reference_position.altitude);
  try {
    double scale;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.point.x, utm_point.point.y, conv_angle,
                                   scale);
    std::string hemisphere;
    if (northp) {
      hemisphere = "N";
    } else {
      hemisphere = "S";
    }
    utm_point.header.frame_id = "utm_" + std::to_string(zone) + hemisphere;
  } catch (GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(e.what());
  }
  return utm_point;
}

/**
 * @brief Get the UTM Position defined within the BasicContainer of the MCM along with the grid-convergence angle
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] mcm MCM to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @param[out] conv_angle grid-convergence angle in degree
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
inline gm::PointStamped getUTMPosition(const MCM& mcm, int& zone, bool& northp, double& conv_angle) {
  return getUTMPosition(mcm.mcm.mcm_parameters.basic_container.reference_position, zone, northp, conv_angle);
}

/**
 * @brief Get the UTM Position defined within the BasicContainer of the MCM
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] mcm MCM to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
inline gm::PointStamped getUTMPosition(const MCM& mcm, int& zone, bool& northp) {
  double conv_angle;  // unused, but required by the function signature
  return getUTMPosition(mcm.mcm.mcm_parameters.basic_container.reference_position, zone, northp, conv_angle);
}

/**
 * @brief Get the UTM Position defined within the BasicContainer of the MCM
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] mcm MCM to get the UTM Position from
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
inline gm::PointStamped getUTMPosition(const MCM& mcm) {
  int zone;           // unused, but required by the function signature
  bool northp;        // unused, but required by the function signature
  double conv_angle;  // unused, but required by the function signature
  return getUTMPosition(mcm.mcm.mcm_parameters.basic_container.reference_position, zone, northp, conv_angle);
}

// ---------- suggested maneuver container getters ----------

/**
 * @brief Retrieves the SuggestedManeuverContainer from the given MCM object.
 *
 * @param mcm The MCM object from which to extract the suggested maneuver container.
 * @return SuggestedManeuverContainer The extracted suggested maneuver container.
 * @throws std::invalid_argument If the maneuver container does not contain a suggested maneuver container.
 */
inline SuggestedManeuverContainer getSuggestedManeuverContainer(const MCM& mcm) {
  if (mcm.mcm.mcm_parameters.maneuver_container.choice != ManeuverContainer::CHOICE_SUGGESTED_MANEUVER_CONTAINER) {
    throw std::invalid_argument("No suggested maneuver container present in MCM");
  }
  return mcm.mcm.mcm_parameters.maneuver_container.suggested_maneuver_container;
}

/**
 * @brief Retrieves the target station ID from a SuggestedManeuverContainer.
 *
 * @param suggested_maneuver_container The container holding the suggested maneuver information.
 * @return The target station ID as a uint32_t.
 */
inline uint32_t getTargetStationId(const SuggestedManeuverContainer& suggested_maneuver_container) {
  return suggested_maneuver_container.target_station_id.value;
}

/**
 * @brief Retrieves the suggested maneuver from a SuggestedManeuverContainer.
 *
 * @param suggested_maneuver_container The container holding the suggested maneuver.
 * @return SuggestedManeuver The suggested maneuver contained in the input.
 * @throws std::invalid_argument If no suggested maneuver is present in the container.
 */
inline SuggestedManeuver getSuggestedManeuver(const SuggestedManeuverContainer& suggested_maneuver_container) {
  if (!suggested_maneuver_container.suggested_maneuver_is_present) {
    throw std::invalid_argument("No suggested maneuver present in SuggestedManeuverContainer");
  }
  return suggested_maneuver_container.suggested_maneuver;
}

/**
 * @brief Retrieves the suggested maneuver from the given MCM message.
 *
 * @param mcm The MCM (Maneuver Coordination Message) object to extract the suggested maneuver from.
 * @return SuggestedManeuver The suggested maneuver contained within the MCM message.
 */
inline SuggestedManeuver getSuggestedManeuver(const MCM& mcm) {
  return getSuggestedManeuver(getSuggestedManeuverContainer(mcm));
}

/**
 * @brief Retrieves the maneuver ID from a SuggestedManeuver object.
 *
 * @param suggested_maneuver The SuggestedManeuver instance to extract the maneuver ID from.
 * @return The maneuver ID as a uint16_t.
 */
inline uint16_t getManeuverId(const SuggestedManeuver& suggested_maneuver) {
  return suggested_maneuver.maneuver_id.value;
}

/**
 * @brief Retrieves the advice update identifier from a SuggestedManeuver object.
 *
 * @param suggested_maneuver The SuggestedManeuver object containing the advice update ID.
 * @return The advice update ID as a uint16_t.
 */
inline uint16_t getAdviceUpdateId(const SuggestedManeuver& suggested_maneuver) {
  return suggested_maneuver.advice_update_id.value;
}

/**
 * @brief Retrieves the confirmation required flag from a SuggestedManeuver.
 *
 * @param suggested_maneuver The SuggestedManeuver object to query.
 * @return true if confirmation is required, false otherwise.
 */
inline bool getConfirmationRequiredFlag(const SuggestedManeuver& suggested_maneuver) {
  return suggested_maneuver.confirmation_required_flag.value;
}

/**
 * @brief Retrieves the ManeuverConstraints from a SuggestedManeuver.
 *
 * @param suggested_maneuver The SuggestedManeuver object from which to extract the constraints.
 * @return ManeuverConstraints The maneuver constraints contained in the SuggestedManeuver.
 * @throws std::invalid_argument If the SuggestedManeuver does not have maneuver constraints present.
 */
inline ManeuverConstraints getManeuverConstraints(const SuggestedManeuver& suggested_maneuver) {
  if (suggested_maneuver.maneuver_parameters.choice != ManeuverParameters::CHOICE_MANEUVER_CONSTRAINTS) {
    throw std::invalid_argument("No maneuver constraints present in SuggestedManeuver");
  }
  return suggested_maneuver.maneuver_parameters.maneuver_constraints;
}

/**
 * @brief Retrieves the maneuver constraints from a given MCM (Maneuver Coordination Message).
 *
 * @param mcm The Maneuver Coordination Message from which to extract constraints.
 * @return ManeuverConstraints The constraints associated with the suggested maneuver.
 */
inline ManeuverConstraints getManeuverConstraints(const MCM& mcm) {
  return getManeuverConstraints(getSuggestedManeuver(mcm));
}

/**
 * @brief Retrieves the list of longitudinal waypoints from the given maneuver constraints.
 *
 * @param maneuver_constraints The ManeuverConstraints object containing the waypoints.
 * @return std::vector<LongitudinalWaypoint> The vector of longitudinal waypoints.
 */
inline std::vector<LongitudinalWaypoint> getLongitudinalWaypoints(const ManeuverConstraints& maneuver_constraints) {
  return maneuver_constraints.longitudinal_maneuver_waypoint_container.array;
}

/**
 * @brief Retrieves the longitudinal waypoints from the given MCM (Maneuver Coordination Message).
 *
 * @param mcm The Maneuver Coordination Message from which to extract waypoints.
 * @return A vector of LongitudinalWaypoint objects representing the waypoints.
 */
inline std::vector<LongitudinalWaypoint> getLongitudinalWaypoints(const MCM& mcm) {
  return getLongitudinalWaypoints(getManeuverConstraints(mcm));
}

/**
 * @brief Converts a LongitudinalWaypoint to a gm::Point.
 *
 * @param longitudinal_waypoint The LongitudinalWaypoint containing x and y distances in centimeters.
 * @return gm::Point The point with x and y coordinates in meters, and z set to 0.0.
 */
inline gm::Point getWaypointDelta(const LongitudinalWaypoint& longitudinal_waypoint) {
  gm::Point point;
  point.x = static_cast<double>(longitudinal_waypoint.waypoint.x_distance.value * 1e-2);  // convert cm to m
  point.y = static_cast<double>(longitudinal_waypoint.waypoint.y_distance.value * 1e-2);  // convert cm to m
  point.z = 0.0;  // z-coordinate is not used in longitudinal waypoints
  return point;
}

/**
 * @brief Returns the minimum arrival time delta for a given longitudinal waypoint.
 *
 * @param longitudinal_waypoint The waypoint containing the minimum arrival time in milliseconds.
 * @return The minimum arrival time delta in seconds.
 */
inline double getMinArrivalTimeDelta(const LongitudinalWaypoint& longitudinal_waypoint) {
  return static_cast<double>(longitudinal_waypoint.min_arrival_time.value * 1e-3);  // convert ms to s
}

/**
 * @brief Returns the maximum allowed arrival time delta for a given longitudinal waypoint.
 *
 * @param longitudinal_waypoint The waypoint containing the maximum arrival time in milliseconds.
 * @return The maximum arrival time delta in seconds.
 */
inline double getMaxArrivalTimeDelta(const LongitudinalWaypoint& longitudinal_waypoint) {
  return static_cast<double>(longitudinal_waypoint.max_arrival_time.value * 1e-3);  // convert ms to s
}

/**
 * @brief Retrieves the minimum velocity from a LongitudinalWaypoint.
 *
 * @param longitudinal_waypoint The LongitudinalWaypoint containing the velocity information in cm/s.
 * @return The minimum velocity in m/s, or NaN if unavailable.
 * @throws std::invalid_argument If the minimum velocity is not present.
 */
inline double getMinVelocity(const LongitudinalWaypoint& longitudinal_waypoint) {
  if (!longitudinal_waypoint.min_velocity_is_present) {
    throw std::invalid_argument("No min velocity present in LongitudinalWaypoint");
  } else if (longitudinal_waypoint.min_velocity.value == SpeedValue::UNAVAILABLE) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(longitudinal_waypoint.min_velocity.value * 1e-2);  // convert cm/s to m/s
}

/**
 * @brief Retrieves the maximum velocity from a LongitudinalWaypoint object.
 *
 * @param longitudinal_waypoint The LongitudinalWaypoint object containing velocity information in cm/s.
 * @return The maximum velocity in m/s, or NaN if unavailable.
 * @throws std::invalid_argument If the maximum velocity is not present.
 */
inline double getMaxVelocity(const LongitudinalWaypoint& longitudinal_waypoint) {
  if (!longitudinal_waypoint.max_velocity_is_present) {
    throw std::invalid_argument("No max velocity present in LongitudinalWaypoint");
  } else if (longitudinal_waypoint.max_velocity.value == SpeedValue::UNAVAILABLE) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(longitudinal_waypoint.max_velocity.value * 1e-2);  // convert cm/s to m/s
}

// ---------- road user container getters ----------

/**
 * @brief Retrieves the RoadUserContainer from the given MCM object.
 *
 * @param mcm The MCM object from which to retrieve the RoadUserContainer.
 * @return RoadUserContainer The extracted RoadUserContainer.
 * @throws std::invalid_argument If the MCM does not contain a RoadUserContainer.
 */
inline RoadUserContainer getRoadUserContainer(const MCM& mcm) {
  if (mcm.mcm.mcm_parameters.maneuver_container.choice != ManeuverContainer::CHOICE_ROAD_USER_CONTAINER) {
    throw std::invalid_argument("No road user container present in MCM");
  }
  return mcm.mcm.mcm_parameters.maneuver_container.road_user_container;
}

}  // namespace etsi_its_mcm_uulm_msgs::access