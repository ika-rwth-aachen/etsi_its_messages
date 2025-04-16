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
 * @file impl/denm/denm_getters.h
 * @brief Getter functions for the ETSI ITS DENM (EN)
 */

#pragma once

namespace etsi_its_denm_msgs::access {

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_getters.h>
#include <etsi_its_msgs_utils/impl/cdd/cdd_v1-3-1_getters.h>

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
 * @brief Get the Heading object
 * 
 * @param denm DENM to get the Heading-Value from
 * @return heading value in degree as decimal number
 */
inline double getHeading(const DENM& denm) {
  if (denm.denm.location_is_present) {
    if (denm.denm.location.event_position_heading_is_present) {
      return getHeadingCDD(denm.denm.location.event_position_heading);
    } else {
      throw std::invalid_argument("Heading is not present!");
    }
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

/**
 * @brief Get the Heading confidence
 * 
 * @param denm DENM to get the Heading-Value from
 * @return standard deviation of heading in degrees as decimal number
 */
inline double getHeadingConfidence(const DENM& denm) {
  if (denm.denm.location_is_present) {
    if (denm.denm.location.event_position_heading_is_present) {
      return getHeadingConfidenceCDD(denm.denm.location.event_position_heading);
    } else {
      throw std::invalid_argument("Heading is not present!");
    }
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

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
 * @brief 
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
 * @brief 
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
 * @brief Get the Cause Code object
 * 
 * @param denm DENM to get the causeCode value from
 * @return causeCode value
 */
inline uint8_t getCauseCode(const DENM& denm) { return denm.denm.situation.event_type.cause_code.value; }

/**
 * @brief Get the Sub Cause Code object
 * 
 * @param denm DENM to get the subCauseCode value from
 * @return subCauseCode value
 */
inline uint8_t getSubCauseCode(const DENM& denm) { return denm.denm.situation.event_type.sub_cause_code.value; }

/**
 * @brief Get the Cause Code Type object
 *
 * https://www.etsi.org/deliver/etsi_en/302600_302699/30263703/01.02.01_30/en_30263703v010201v.pdf
 * 
 * @param denm DENM to get the causeCodeType value from
 * @return causeCodeType value
 */
inline std::string getCauseCodeType(const DENM& denm) {
  if (denm.denm.situation_is_present) {
    int cause_code = getCauseCode(denm);
    std::string cause_code_type = "undefined";

    if (cause_code == CauseCodeType().TRAFFIC_CONDITION)
      cause_code_type = "traffic condition";
    else if (cause_code == CauseCodeType().ACCIDENT)
      cause_code_type = "accident";
    else if (cause_code == CauseCodeType().ROADWORKS)
      cause_code_type = "roadworks";
    else if (cause_code == CauseCodeType().IMPASSABILITY)
      cause_code_type = "impassibility";
    else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_ADHESION)
      cause_code_type = "adverse weather condition - adhesion";
    else if (cause_code == CauseCodeType().AQUAPLANNNING)
      cause_code_type = "aquaplanning";
    else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_SURFACE_CONDITION)
      cause_code_type = "hazardous location - surface condition";
    else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD)
      cause_code_type = "hazardous location - obstacle on the road";
    else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD)
      cause_code_type = "hazardous location - animal on the road";
    else if (cause_code == CauseCodeType().HUMAN_PRESENCE_ON_THE_ROAD)
      cause_code_type = "human presence on the road";
    else if (cause_code == CauseCodeType().WRONG_WAY_DRIVING)
      cause_code_type = "wrong way driving";
    else if (cause_code == CauseCodeType().RESCUE_AND_RECOVERY_WORK_IN_PROGRESS)
      cause_code_type = "rescue and recovery in progress";
    else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION)
      cause_code_type = "adverse weather condition - extreme weather condition";
    else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_VISIBILITY)
      cause_code_type = "adverse weather condition - visibility";
    else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_PRECIPITATION)
      cause_code_type = "adverse weather condition - precipitation";
    else if (cause_code == CauseCodeType().SLOW_VEHICLE)
      cause_code_type = "slow vehicle";
    else if (cause_code == CauseCodeType().DANGEROUS_END_OF_QUEUE)
      cause_code_type = "dangerous end of queue";
    else if (cause_code == CauseCodeType().VEHICLE_BREAKDOWN)
      cause_code_type = "vehicle breakdown";
    else if (cause_code == CauseCodeType().POST_CRASH)
      cause_code_type = "post crash";
    else if (cause_code == CauseCodeType().HUMAN_PROBLEM)
      cause_code_type = "human problem";
    else if (cause_code == CauseCodeType().STATIONARY_VEHICLE)
      cause_code_type = "stationary vehicle";
    else if (cause_code == CauseCodeType().EMERGENCY_VEHICLE_APPROACHING)
      cause_code_type = "emergency vehicle approaching";
    else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_DANGEROUS_CURVE)
      cause_code_type = "hazardous location - dangerous curve";
    else if (cause_code == CauseCodeType().COLLISION_RISK)
      cause_code_type = "collision risk";
    else if (cause_code == CauseCodeType().SIGNAL_VIOLATION)
      cause_code_type = "signal violation";
    else if (cause_code == CauseCodeType().DANGEROUS_SITUATION)
      cause_code_type = "dangerous situation";

    return cause_code_type;
  } else {
    throw std::invalid_argument("SituationContainer is not present!");
  }
}

/**
 * @brief Get the Sub Cause Code Type object
 *
 * https://www.etsi.org/deliver/etsi_en/302600_302699/30263703/01.02.01_30/en_30263703v010201v.pdf
 * 
 * @param denm DENM to get the subCauseCodeType value from
 * @return causeCodeType value 
 */
inline std::string getSubCauseCodeType(const DENM& denm) {
  if (denm.denm.situation_is_present) {
    int cause_code = getCauseCode(denm);
    int sub_cause_code = getSubCauseCode(denm);
    std::string sub_cause_code_type = "undefined";
    if (cause_code == CauseCodeType().TRAFFIC_CONDITION) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "not defined";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "traffic jam slowly increasing";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "traffic jam increasing";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "traffic jam strongly increasing";
      else if (sub_cause_code == 5)
        sub_cause_code_type = "traffic stationary";
      else if (sub_cause_code == 6)
        sub_cause_code_type = "traffic jam slightly decreasing";
      else if (sub_cause_code == 7)
        sub_cause_code_type = "traffic jam decreasing";
      else if (sub_cause_code == 8)
        sub_cause_code_type = "traffic jam strongly decreasing";
    } else if (cause_code == CauseCodeType().ACCIDENT) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 7)
        sub_cause_code_type = "not defined";
      else if (sub_cause_code == 8)
        sub_cause_code_type = "assistance requested (e-Call)";
    } else if (cause_code == CauseCodeType().ROADWORKS) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 3)
        sub_cause_code_type = "not defined";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "short-term stationary roadworks";
      else if (sub_cause_code == 5)
        sub_cause_code_type = "street cleaning";
      else if (sub_cause_code == 6)
        sub_cause_code_type = "winter service";
    } else if (cause_code == CauseCodeType().IMPASSABILITY)
      sub_cause_code_type = "not defined";
    else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_ADHESION) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 10)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().AQUAPLANNNING)
      sub_cause_code_type = "not defined";
    else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_SURFACE_CONDITION) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 9)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 7)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 4)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().HUMAN_PRESENCE_ON_THE_ROAD) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 3)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().WRONG_WAY_DRIVING) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "vehicle driving in wrong lane";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "vehicle driving in wrong driving direction";
    } else if (cause_code == CauseCodeType().RESCUE_AND_RECOVERY_WORK_IN_PROGRESS) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 5)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 6)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_VISIBILITY) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 8)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().ADVERSE_WEATHER_CONDITION_PRECIPITATION) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 3)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().SLOW_VEHICLE) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 8)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().DANGEROUS_END_OF_QUEUE) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code >= 1 && sub_cause_code <= 8)
        sub_cause_code_type = "not defined";
    } else if (cause_code == CauseCodeType().VEHICLE_BREAKDOWN) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "lack of fuel";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "lack of battery";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "engine problem";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "transmission problem";
      else if (sub_cause_code == 5)
        sub_cause_code_type = "engine cooling problem";
      else if (sub_cause_code == 6)
        sub_cause_code_type = "braking system problem";
      else if (sub_cause_code == 7)
        sub_cause_code_type = "steering problem";
      else if (sub_cause_code == 8)
        sub_cause_code_type = "tyre puncture";
    } else if (cause_code == CauseCodeType().POST_CRASH) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "accident without e-Call triggered";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "accident with e-Call manually triggered";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "accident with e-Call automatical triggered";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "accident with e-Call triggered without a possible access to a cell network";
    } else if (cause_code == CauseCodeType().HUMAN_PROBLEM) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "glycaemia problem";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "heart problem";
    } else if (cause_code == CauseCodeType().STATIONARY_VEHICLE) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "human problem";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "vehicle breakdown";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "post crash";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "public transport stop";
      else if (sub_cause_code == 5)
        sub_cause_code_type = "carrying dangerous goods";
    } else if (cause_code == CauseCodeType().EMERGENCY_VEHICLE_APPROACHING) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "emergency vehicle approaching";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "prioritized vehicle approaching";
    } else if (cause_code == CauseCodeType().HAZARDOUS_LOCATION_DANGEROUS_CURVE) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "dangerous left turn curve";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "dangerous right turn curve";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "multiple curves starting with unknown turning direction";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "multiple curves starting with left turn";
      else if (sub_cause_code == 5)
        sub_cause_code_type = "multiple curves starting with right turn";
    } else if (cause_code == CauseCodeType().COLLISION_RISK) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "longitudinal collision risk";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "crossing collision risk";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "lateral collision risk";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "collision risk involving vulnerable road user";
    } else if (cause_code == CauseCodeType().SIGNAL_VIOLATION) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "stop sign violation";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "traffic light violation";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "turning regulation violation";
    } else if (cause_code == CauseCodeType().DANGEROUS_SITUATION) {
      if (sub_cause_code == 0)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == 1)
        sub_cause_code_type = "emergency electronic break lights";
      else if (sub_cause_code == 2)
        sub_cause_code_type = "pre-crash system activated";
      else if (sub_cause_code == 3)
        sub_cause_code_type = "ESP (electronic stability program) activated";
      else if (sub_cause_code == 4)
        sub_cause_code_type = "ABS (anti-lock breaking system) activated";
      else if (sub_cause_code == 5)
        sub_cause_code_type = "AEB (automatic emergency breaking) activated";
      else if (sub_cause_code == 6)
        sub_cause_code_type = "break warning activated";
      else if (sub_cause_code == 7)
        sub_cause_code_type = "collision risk warning activated";
    }
    return sub_cause_code_type;
  } else {
    throw std::invalid_argument("SituationContainer is not present!");
  }
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

}  // namespace etsi_its_denm_msgs::access
