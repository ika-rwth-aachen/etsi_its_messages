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
 * @file impl/denm/denm_getters_common.h
 * @brief Common getter functions for the ETSI ITS DENM (EN and TS)
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_DENM_DENM_GETTERS_COMMON_H
#define ETSI_ITS_MSGS_UTILS_IMPL_DENM_DENM_GETTERS_COMMON_H

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_getters.h>

/**
 * @brief Get the Station ID object
 *
 * @param denm DENM to get the StationID value from
 * @return stationID value
 */
inline uint32_t getStationID(const DENM& denm) { return getStationID(denm.header); }

/**
 * @brief Get the Reference Time object
 * 
 * @param denm DENM to get the ReferenceTime-Value from
 * @return TimestampIts 
 */
inline TimestampIts getReferenceTime(const DENM& denm) { return denm.denm.management.reference_time; }

/**
 * @brief Get the ReferenceTime-Value
 * 
 * @param denm DENM to get the ReferenceTime-Value from 
 * @return uint64_t the ReferenceTime-Value
 */
inline uint64_t getReferenceTimeValue(const DENM& denm) { return getReferenceTime(denm).value; }

/**
 * @brief Get the stationType object
 * 
 * @param denm DENM to get the stationType value from
 * @return stationType value
 */
inline uint8_t getStationType(const DENM& denm) { return denm.denm.management.station_type.value; }

/**
 * @brief Get the Latitude value of DENM
 * 
 * @param denm DENM to get the Latitude value from
 * @return Latitude value in degree as decimal number
 */
inline double getLatitude(const DENM& denm) { return getLatitude(denm.denm.management.event_position.latitude); }

/**
 * @brief Get the Longitude value of DENM
 * 
 * @param denm DENM to get the Longitude value from
 * @return Longitude value in degree as decimal number
 */
inline double getLongitude(const DENM& denm) { return getLongitude(denm.denm.management.event_position.longitude); }

/**
 * @brief Get the Altitude value of DENM
 * 
 * @param denm DENM to get the Altitude value from
 * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline double getAltitude(const DENM& denm) { return getAltitude(denm.denm.management.event_position.altitude); }

/**
 * @brief Get the IsHeadingPresent object
 * 
 * @param denm DENM to get the IsHeadingPresent-Value from
 * @return IsHeadingPresent-Value (true or false)
 */
inline bool getIsHeadingPresent(const DENM& denm) {
  if (denm.denm.location_is_present) {
    return denm.denm.location.event_position_heading_is_present;
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

/**
 * @brief Get the vehicle speed
 * 
 * @param denm DENM to get the speed value from
 * @return speed value in m/s as decimal number
 */
inline double getSpeed(const DENM& denm) {
  if (denm.denm.location_is_present) {
    if (denm.denm.location.event_speed_is_present) {
      return getSpeed(denm.denm.location.event_speed);
    } else {
      throw std::invalid_argument("Speed is not present!");
    }
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

/**
 * @brief Get the IsSpeedPresent object
 * 
 * @param denm DENM to get the IsSpeedPresent-Value from
 * @return IsSpeedPresent-Value (true or false)
 */
inline bool getIsSpeedPresent(const DENM& denm) {
  if (denm.denm.location_is_present) {
    return denm.denm.location.event_speed_is_present;
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

/**
 * @brief Get the Speed Confidence
 * 
 * @param denm DENM to get the Speed Confidence from
 * @return double standard deviation of the speed in m/s as decimal number
 */
inline double getSpeedConfidence(const DENM& denm) {
  return getSpeedConfidence(
    denm.denm.location.event_speed);
}

/**
 * @brief Get the UTM Position defined within the ManagementContainer of the DENM
 * 
 * @param denm DENM to get the UTM Position from
 * @param zone the UTM zone (zero means UPS)
 * @param northp hemisphere (true means north, false means south)
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
inline gm::PointStamped getUTMPosition(const DENM& denm, int& zone, bool& northp) {
  return getUTMPosition(denm.denm.management.event_position, zone, northp);
}

/**
 * @brief Get the UTM Position defined within the ManagementContainer of the DENM
 * 
 * @param denm DENM to get the UTM Position from
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
inline gm::PointStamped getUTMPosition(const DENM& denm) {
  int zone;
  bool northp;
  return getUTMPosition(denm.denm.management.event_position, zone, northp);
}

/**
 * @brief Get the Driving Lane Status in form of bool vector
 *
 * @param driving_lane_status
 * @return std::vector<bool>
 */
inline std::vector<bool> getDrivingLaneStatus(const DrivingLaneStatus& driving_lane_status) {
  return getBitString(driving_lane_status.value, driving_lane_status.bits_unused);
}

/**
 * @brief Get the Lightbar Siren In Use in form of bool vector
 *
 * @param light_bar_siren_in_use
 * @return std::vector<bool>
 */
inline std::vector<bool> getLightBarSirenInUse(const LightBarSirenInUse& light_bar_siren_in_use) {
  return getBitString(light_bar_siren_in_use.value, light_bar_siren_in_use.bits_unused);
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_DENM_DENM_GETTERS_COMMON_H