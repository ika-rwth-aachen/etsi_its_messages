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
 * @file impl/denm/denm_ts_getters.h
 * @brief Getter functions for the ETSI ITS DENM (TS)
 */

#pragma once

namespace etsi_its_denm_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-2-1_getters.h>

#include <etsi_its_msgs_utils/impl/denm/denm_getters_common.h>

/**
 * @brief Get the WGS Heading object
 * 
 * @param denm DENM to get the WGS Heading-Value from
 * @return heading value in degree as decimal number
 */
inline double getWGSHeading(const DENM& denm) {
  if (denm.denm.location_is_present) {
    if (denm.denm.location.event_position_heading_is_present) {
      return getWGSHeadingCDD(denm.denm.location.event_position_heading);
    } else {
      throw std::invalid_argument("Wgs84Angle is not present!");
    }
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

/**
 * @brief Get the WGS Heading confidence
 * 
 * @param denm DENM to get the WGSHeading-Value from
 * @return standard deviation of heading in degrees as decimal number
 */
inline double getWGSHeadingConfidence(const DENM& denm) {
  if (denm.denm.location_is_present) {
    if (denm.denm.location.event_position_heading_is_present) {
      return getWGSHeadingConfidenceCDD(denm.denm.location.event_position_heading);
    } else {
      throw std::invalid_argument("Wgs84Angle is not present!");
    }
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

/**
 * @brief Get the Cause Code object
 * 
 * @param denm DENM to get the causeCode value from
 * @return causeCode value
 */
inline uint8_t getCauseCode(const DENM& denm) { return denm.denm.situation.event_type.cc_and_scc.choice; }

/**
 * @brief Get the Sub Cause Code object
 * 
 * @param denm DENM to get the subCauseCode value from
 * @return subCauseCode value
 */
inline uint8_t getSubCauseCode(const DENM& denm) {
  int cause_code = getCauseCode(denm);
  if (cause_code == CauseCodeChoice().CHOICE_TRAFFIC_CONDITION1)
    return denm.denm.situation.event_type.cc_and_scc.traffic_condition1.value;
  else if (cause_code == CauseCodeChoice().CHOICE_ACCIDENT2)
    return denm.denm.situation.event_type.cc_and_scc.accident2.value;
  else if (cause_code == CauseCodeChoice().CHOICE_ROADWORKS3)
    return denm.denm.situation.event_type.cc_and_scc.roadworks3.value;
  else if (cause_code == CauseCodeChoice().CHOICE_IMPASSABILITY5)
    return denm.denm.situation.event_type.cc_and_scc.impassability5.value;
  else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_ADHESION6)
    return denm.denm.situation.event_type.cc_and_scc.adverse_weather_condition_adhesion6.value;
  else if (cause_code == CauseCodeChoice().CHOICE_AQUAPLANING7)
    return denm.denm.situation.event_type.cc_and_scc.aquaplaning7.value;
  else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_SURFACE_CONDITION9)
    return denm.denm.situation.event_type.cc_and_scc.hazardous_location_surface_condition9.value;
  else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD10)
    return denm.denm.situation.event_type.cc_and_scc.hazardous_location_obstacle_on_the_road10.value;
  else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD11)
    return denm.denm.situation.event_type.cc_and_scc.hazardous_location_animal_on_the_road11.value;
  else if (cause_code == CauseCodeChoice().CHOICE_HUMAN_PRESENCE_ON_THE_ROAD12)
    return denm.denm.situation.event_type.cc_and_scc.human_presence_on_the_road12.value;
  else if (cause_code == CauseCodeChoice().CHOICE_WRONG_WAY_DRIVING14)
    return denm.denm.situation.event_type.cc_and_scc.wrong_way_driving14.value;
  else if (cause_code == CauseCodeChoice().CHOICE_RESCUE_AND_RECOVERY_WORK_IN_PROGRESS15)
    return denm.denm.situation.event_type.cc_and_scc.rescue_and_recovery_work_in_progress15.value;
  else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION17)
    return denm.denm.situation.event_type.cc_and_scc.adverse_weather_condition_extreme_weather_condition17.value;
  else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_VISIBILITY18)
    return denm.denm.situation.event_type.cc_and_scc.adverse_weather_condition_visibility18.value;
  else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_PRECIPITATION19)
    return denm.denm.situation.event_type.cc_and_scc.adverse_weather_condition_precipitation19.value;
  else if (cause_code == CauseCodeChoice().CHOICE_VIOLENCE20)
    return denm.denm.situation.event_type.cc_and_scc.violence20.value;
  else if (cause_code == CauseCodeChoice().CHOICE_SLOW_VEHICLE26)
    return denm.denm.situation.event_type.cc_and_scc.slow_vehicle26.value;
  else if (cause_code == CauseCodeChoice().CHOICE_DANGEROUS_END_OF_QUEUE27)
    return denm.denm.situation.event_type.cc_and_scc.dangerous_end_of_queue27.value;
  else if (cause_code == CauseCodeChoice().CHOICE_PUBLIC_TRANSPORT_VEHICLE_APPROACHING28)
    return denm.denm.situation.event_type.cc_and_scc.public_transport_vehicle_approaching28.value;
  else if (cause_code == CauseCodeChoice().CHOICE_VEHICLE_BREAKDOWN91)
    return denm.denm.situation.event_type.cc_and_scc.vehicle_breakdown91.value;
  else if (cause_code == CauseCodeChoice().CHOICE_POST_CRASH92)
    return denm.denm.situation.event_type.cc_and_scc.post_crash92.value;
  else if (cause_code == CauseCodeChoice().CHOICE_HUMAN_PROBLEM93)
    return denm.denm.situation.event_type.cc_and_scc.human_problem93.value;
  else if (cause_code == CauseCodeChoice().CHOICE_STATIONARY_VEHICLE94)
    return denm.denm.situation.event_type.cc_and_scc.stationary_vehicle94.value;
  else if (cause_code == CauseCodeChoice().CHOICE_EMERGENCY_VEHICLE_APPROACHING95)
    return denm.denm.situation.event_type.cc_and_scc.emergency_vehicle_approaching95.value;
  else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_DANGEROUS_CURVE96)
    return denm.denm.situation.event_type.cc_and_scc.hazardous_location_dangerous_curve96.value;
  else if (cause_code == CauseCodeChoice().CHOICE_COLLISION_RISK97)
    return denm.denm.situation.event_type.cc_and_scc.collision_risk97.value;
  else if (cause_code == CauseCodeChoice().CHOICE_SIGNAL_VIOLATION98)
    return denm.denm.situation.event_type.cc_and_scc.signal_violation98.value;
  else if (cause_code == CauseCodeChoice().CHOICE_DANGEROUS_SITUATION99)
    return denm.denm.situation.event_type.cc_and_scc.dangerous_situation99.value;
  else if (cause_code == CauseCodeChoice().CHOICE_RAILWAY_LEVEL_CROSSING100)
    return denm.denm.situation.event_type.cc_and_scc.railway_level_crossing100.value;
  return denm.denm.situation.event_type.cc_and_scc.reserved0.value;
}

/**
 * @brief Get the Cause Code Type object
 *
 * https://www.etsi.org/deliver/etsi_ts/103800_103899/103831/02.02.01_60/ts_103831v020201p.pdf
 * 
 * @param denm DENM to get the causeCodeType value from
 * @return causeCodeType value
 */
inline std::string getCauseCodeType(const DENM& denm) {
  if (denm.denm.situation_is_present) {
    int cause_code = getCauseCode(denm);
    std::string cause_code_type = "undefined";

    if (cause_code == CauseCodeChoice().CHOICE_TRAFFIC_CONDITION1)
      cause_code_type = "traffic condition";
    else if (cause_code == CauseCodeChoice().CHOICE_ACCIDENT2)
      cause_code_type = "accident";
    else if (cause_code == CauseCodeChoice().CHOICE_ROADWORKS3)
      cause_code_type = "roadworks";
    else if (cause_code == CauseCodeChoice().CHOICE_IMPASSABILITY5)
      cause_code_type = "impassability";
    else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_ADHESION6)
      cause_code_type = "adverse weather condition - adhesion";
    else if (cause_code == CauseCodeChoice().CHOICE_AQUAPLANING7)
      cause_code_type = "aquaplaning";
    else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_SURFACE_CONDITION9)
      cause_code_type = "hazardous location - surface condition";
    else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD10)
      cause_code_type = "hazardous location - obstacle on the road";
    else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD11)
      cause_code_type = "hazardous location - animal on the road";
    else if (cause_code == CauseCodeChoice().CHOICE_HUMAN_PRESENCE_ON_THE_ROAD12)
      cause_code_type = "human presence on the road";
    else if (cause_code == CauseCodeChoice().CHOICE_WRONG_WAY_DRIVING14)
      cause_code_type = "wrong way driving";
    else if (cause_code == CauseCodeChoice().CHOICE_RESCUE_AND_RECOVERY_WORK_IN_PROGRESS15)
      cause_code_type = "rescue and recovery in progress";
    else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION17)
      cause_code_type = "adverse weather condition - extreme weather condition";
    else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_VISIBILITY18)
      cause_code_type = "adverse weather condition - visibility";
    else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_PRECIPITATION19)
      cause_code_type = "adverse weather condition - precipitation";
    else if (cause_code == CauseCodeChoice().CHOICE_VIOLENCE20)
      cause_code_type = "violence";
    else if (cause_code == CauseCodeChoice().CHOICE_SLOW_VEHICLE26)
      cause_code_type = "slow vehicle";
    else if (cause_code == CauseCodeChoice().CHOICE_DANGEROUS_END_OF_QUEUE27)
      cause_code_type = "dangerous end of queue";
    else if (cause_code == CauseCodeChoice().CHOICE_PUBLIC_TRANSPORT_VEHICLE_APPROACHING28)
      cause_code_type = "public transport vehicle approaching";
    else if (cause_code == CauseCodeChoice().CHOICE_VEHICLE_BREAKDOWN91)
      cause_code_type = "vehicle breakdown";
    else if (cause_code == CauseCodeChoice().CHOICE_POST_CRASH92)
      cause_code_type = "post crash";
    else if (cause_code == CauseCodeChoice().CHOICE_HUMAN_PROBLEM93)
      cause_code_type = "human problem";
    else if (cause_code == CauseCodeChoice().CHOICE_STATIONARY_VEHICLE94)
      cause_code_type = "stationary vehicle";
    else if (cause_code == CauseCodeChoice().CHOICE_EMERGENCY_VEHICLE_APPROACHING95)
      cause_code_type = "emergency vehicle approaching";
    else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_DANGEROUS_CURVE96)
      cause_code_type = "hazardous location - dangerous curve";
    else if (cause_code == CauseCodeChoice().CHOICE_COLLISION_RISK97)
      cause_code_type = "collision risk";
    else if (cause_code == CauseCodeChoice().CHOICE_SIGNAL_VIOLATION98)
      cause_code_type = "signal violation";
    else if (cause_code == CauseCodeChoice().CHOICE_DANGEROUS_SITUATION99)
      cause_code_type = "dangerous situation";
    else if (cause_code == CauseCodeChoice().CHOICE_RAILWAY_LEVEL_CROSSING100)
      cause_code_type = "railway level crossing";

    return cause_code_type;
  } else {
    throw std::invalid_argument("SituationContainer is not present!");
  }
}

/**
 * @brief Get the Sub Cause Code Type object
 *
 * https://www.etsi.org/deliver/etsi_ts/103800_103899/103831/02.02.01_60/ts_103831v020201p.pdf
 * 
 * @param denm DENM to get the subCauseCodeType value from
 * @return causeCodeType value 
 */
inline std::string getSubCauseCodeType(const DENM& denm) {
  if (denm.denm.situation_is_present) {
    int cause_code = getCauseCode(denm);
    int sub_cause_code = getSubCauseCode(denm);
    std::string sub_cause_code_type = "undefined";
    if (cause_code == CauseCodeChoice().CHOICE_TRAFFIC_CONDITION1) {
      if (sub_cause_code == TrafficConditionSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == TrafficConditionSubCauseCode().INCREASED_VOLUME_OF_TRAFFIC)
        sub_cause_code_type = "increased volume of traffic";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM_SLOWLY_INCREASING)
        sub_cause_code_type = "traffic jam slowly increasing";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM_INCREASING)
        sub_cause_code_type = "traffic jam increasing";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM_STRONGLY_INCREASING)
        sub_cause_code_type = "traffic jam strongly increasing";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM)
        sub_cause_code_type = "traffic jam";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM_SLIGHTLY_DECREASING)
        sub_cause_code_type = "traffic jam slightly decreasing";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM_DECREASING)
        sub_cause_code_type = "traffic jam decreasing";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM_STRONGLY_DECREASING)
        sub_cause_code_type = "traffic jam strongly decreasing";
      else if (sub_cause_code == TrafficConditionSubCauseCode().TRAFFIC_JAM_STABLE)
        sub_cause_code_type = "traffic jam stable";
    } else if (cause_code == CauseCodeChoice().CHOICE_ACCIDENT2) {
      if (sub_cause_code == AccidentSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == AccidentSubCauseCode().MULTI_VEHICLE_ACCIDENT)
        sub_cause_code_type = "multi-vehicle accident";
      else if (sub_cause_code == AccidentSubCauseCode().HEAVY_ACCIDENT)
        sub_cause_code_type = "heavy accident";
      else if (sub_cause_code == AccidentSubCauseCode().ACCIDENT_INVOLVING_LORRY)
        sub_cause_code_type = "accident involving lorry";
      else if (sub_cause_code == AccidentSubCauseCode().ACCIDENT_INVOLVING_BUS)
        sub_cause_code_type = "accident involving bus";
      else if (sub_cause_code == AccidentSubCauseCode().ACCIDENT_INVOLVING_HAZARDOUS_MATERIALS)
        sub_cause_code_type = "accident involving hazardous materials";
      else if (sub_cause_code == AccidentSubCauseCode().ACCIDENT_ON_OPPOSITE_LANE)
        sub_cause_code_type = "accident on opposite lane";
      else if (sub_cause_code == AccidentSubCauseCode().UNSECURED_ACCIDENT)
        sub_cause_code_type = "unsecured accident";
      else if (sub_cause_code == AccidentSubCauseCode().ASSISTANCE_REQUESTED)
        sub_cause_code_type = "assistance requested (e-Call)";
    } else if (cause_code == CauseCodeChoice().CHOICE_ROADWORKS3) {
      if (sub_cause_code == RoadworksSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == RoadworksSubCauseCode().MAJOR_ROADWORKS)
        sub_cause_code_type = "major roadworks";
      else if (sub_cause_code == RoadworksSubCauseCode().ROAD_MARKING_WORK)
        sub_cause_code_type = "road marking work";
      else if (sub_cause_code == RoadworksSubCauseCode().SLOW_MOVING_ROAD_MAINTENANCE)
        sub_cause_code_type = "slow moving road maintenance";
      else if (sub_cause_code == RoadworksSubCauseCode().SHORT_TERM_STATIONARY_ROADWORKS)
        sub_cause_code_type = "short-term stationary roadworks";
      else if (sub_cause_code == RoadworksSubCauseCode().STREET_CLEANING)
        sub_cause_code_type = "street cleaning";
      else if (sub_cause_code == RoadworksSubCauseCode().WINTER_SERVICE)
        sub_cause_code_type = "winter service";
    } else if (cause_code == CauseCodeChoice().CHOICE_IMPASSABILITY5) {
      if (sub_cause_code == ImpassabilitySubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == ImpassabilitySubCauseCode().FLOODING)
        sub_cause_code_type = "flooding";
      else if (sub_cause_code == ImpassabilitySubCauseCode().DANGER_OF_AVALANCHES)
        sub_cause_code_type = "danger of avalanches";
      else if (sub_cause_code == ImpassabilitySubCauseCode().BLASTING_OF_AVALANCHES)
        sub_cause_code_type = "blasting of avalanches";
      else if (sub_cause_code == ImpassabilitySubCauseCode().LANDSLIPS)
        sub_cause_code_type = "landslips";
      else if (sub_cause_code == ImpassabilitySubCauseCode().CHEMICAL_SPILLAGE)
        sub_cause_code_type = "chemical spillage";
      else if (sub_cause_code == ImpassabilitySubCauseCode().WINTER_CLOSURE)
        sub_cause_code_type = "winter closure";
      else if (sub_cause_code == ImpassabilitySubCauseCode().SINKHOLE)
        sub_cause_code_type = "sinkhole";
      else if (sub_cause_code == ImpassabilitySubCauseCode().EARTHQUAKE_DAMAGE)
        sub_cause_code_type = "earthquake damage";
      else if (sub_cause_code == ImpassabilitySubCauseCode().FALLEN_TREES)
        sub_cause_code_type = "fallen trees";
      else if (sub_cause_code == ImpassabilitySubCauseCode().ROCKFALLS)
        sub_cause_code_type = "rockfalls";
      else if (sub_cause_code == ImpassabilitySubCauseCode().SEWER_OVERFLOW)
        sub_cause_code_type = "sewer overflow";
      else if (sub_cause_code == ImpassabilitySubCauseCode().STORM_DAMAGE)
        sub_cause_code_type = "storm damage";
      else if (sub_cause_code == ImpassabilitySubCauseCode().SUBSIDENCE)
        sub_cause_code_type = "subsidence";
      else if (sub_cause_code == ImpassabilitySubCauseCode().BURST_PIPE)
        sub_cause_code_type = "burst pipe";
      else if (sub_cause_code == ImpassabilitySubCauseCode().BURST_WATER_MAIN)
        sub_cause_code_type = "burst water main";
      else if (sub_cause_code == ImpassabilitySubCauseCode().FALLEN_POWER_CABLES)
        sub_cause_code_type = "fallen power cables";
      else if (sub_cause_code == ImpassabilitySubCauseCode().SNOW_DRIFTS)
        sub_cause_code_type = "snow drifts";
    } else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_ADHESION6) {
      if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().HEAVY_FROST_ON_ROAD)
        sub_cause_code_type = "heavy frost on road";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().FUEL_ON_ROAD)
        sub_cause_code_type = "fuel on road";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().MUD_ON_ROAD)
        sub_cause_code_type = "mud on road";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().SNOW_ON_ROAD)
        sub_cause_code_type = "snow on road";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().ICE_ON_ROAD)
        sub_cause_code_type = "ice on road";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().BLACK_ICE_ON_ROAD)
        sub_cause_code_type = "black ice on road";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().OIL_ON_ROAD)
        sub_cause_code_type = "oil on road";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().LOOSE_CHIPPINGS)
        sub_cause_code_type = "loose chippings";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().INSTANT_BLACK_ICE)
        sub_cause_code_type = "instant black ice";
      else if (sub_cause_code == AdverseWeatherConditionAdhesionSubCauseCode().ROADS_SALTED)
        sub_cause_code_type = "roads salted";
    } else if (cause_code == CauseCodeChoice().CHOICE_AQUAPLANING7)
      sub_cause_code_type = "not defined";
    else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_SURFACE_CONDITION9) {
      if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().ROCKFALLS)
        sub_cause_code_type = "rockfalls";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().EARTHQUAKE_DAMAGE)
        sub_cause_code_type = "earthquake damage";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().SEWER_COLLAPSE)
        sub_cause_code_type = "sewer collapse";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().SUBSIDENCE)
        sub_cause_code_type = "subsidence";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().SNOW_DRIFTS)
        sub_cause_code_type = "snow drifts";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().STORM_DAMAGE)
        sub_cause_code_type = "storm damage";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().BURST_PIPE)
        sub_cause_code_type = "burst pipe";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().VOLCANO_ERUPTION)
        sub_cause_code_type = "volcano eruption";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().FALLING_ICE)
        sub_cause_code_type = "falling ice";
      else if (sub_cause_code == HazardousLocationSurfaceConditionSubCauseCode().FIRE)
        sub_cause_code_type = "fire";
    } else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD10) {
      if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().SHED_LOAD)
        sub_cause_code_type = "shed load";
      else if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().PARTS_OF_VEHICLES)
        sub_cause_code_type = "parts of vehicles";
      else if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().PARTS_OF_TYRES)
        sub_cause_code_type = "parts of tyres";
      else if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().BIG_OBJECTS)
        sub_cause_code_type = "big objects";
      else if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().FALLEN_TREES)
        sub_cause_code_type = "fallen trees";
      else if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().HUB_CAPS)
        sub_cause_code_type = "hub caps";
      else if (sub_cause_code == HazardousLocationObstacleOnTheRoadSubCauseCode().WAITING_VEHICLES)
        sub_cause_code_type = "waiting vehicles";
    } else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD11) {
      if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().WILD_ANIMALS)
        sub_cause_code_type = "wild animals";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().HERD_OF_ANIMALS)
        sub_cause_code_type = "herd of animals";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().SMALL_ANIMALS)
        sub_cause_code_type = "small animals";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().LARGE_ANIMALS)
        sub_cause_code_type = "large animals";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().WILD_ANIMALS_SMALL)
        sub_cause_code_type = "wild animals small";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().WILD_ANIMALS_LARGE)
        sub_cause_code_type = "wild animals large";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().DOMESTIC_ANIMALS)
        sub_cause_code_type = "domestic animals";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().DOMESTIC_ANIMALS_SMALL)
        sub_cause_code_type = "domestic animals small";
      else if (sub_cause_code == HazardousLocationAnimalOnTheRoadSubCauseCode().DOMESTIC_ANIMALS_LARGE)
        sub_cause_code_type = "domestic animals large";
    } else if (cause_code == CauseCodeChoice().CHOICE_HUMAN_PRESENCE_ON_THE_ROAD12) {
      if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().CHILDREN_ON_ROADWAY)
        sub_cause_code_type = "children on roadway";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().CYCLIST_ON_ROADWAY)
        sub_cause_code_type = "cyclist on roadway";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().MOTORCYCLIST_ON_ROADWAY)
        sub_cause_code_type = "motorcyclist on roadway";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().PEDESTRIAN)
        sub_cause_code_type = "pedestrian";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().ORDINARY_PEDESTRIAN)
        sub_cause_code_type = "ordinary pedestrian";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().ROAD_WORKER)
        sub_cause_code_type = "road worker";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().FIRST_RESPONDER)
        sub_cause_code_type = "first responder";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().LIGHT_VRU_VEHICLE)
        sub_cause_code_type = "light VRU vehicle";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().BICYCLIST)
        sub_cause_code_type = "bicyclist";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().WHEELCHAIR_USER)
        sub_cause_code_type = "wheelchair user";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().HORSE_AND_RIDER)
        sub_cause_code_type = "horse and rider";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().ROLLERSKATER)
        sub_cause_code_type = "rollerskater";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().E_SCOOTER)
        sub_cause_code_type = "e-scooter";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().PERSONAL_TRANSPORTER)
        sub_cause_code_type = "personal transporter";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().PEDELEC)
        sub_cause_code_type = "pedelec";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().SPEED_PEDELEC)
        sub_cause_code_type = "speed pedelec";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().PTW)
        sub_cause_code_type = "PTW";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().MOPED)
        sub_cause_code_type = "moped";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().MOTORCYCLE)
        sub_cause_code_type = "motorcycle";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().MOTORCYCLE_AND_SIDECAR_RIGHT)
        sub_cause_code_type = "motorcycle and sidecar right";
      else if (sub_cause_code == HumanPresenceOnTheRoadSubCauseCode().MOTORCYCLE_AND_SIDECAR_LEFT)
        sub_cause_code_type = "motorcycle and sidecar left";
    } else if (cause_code == CauseCodeChoice().CHOICE_WRONG_WAY_DRIVING14) {
      if (sub_cause_code == WrongWayDrivingSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == WrongWayDrivingSubCauseCode().WRONG_LANE)
        sub_cause_code_type = "vehicle driving in wrong lane";
      else if (sub_cause_code == WrongWayDrivingSubCauseCode().WRONG_DIRECTION)
        sub_cause_code_type = "vehicle driving in wrong driving direction";
    } else if (cause_code == CauseCodeChoice().CHOICE_RESCUE_AND_RECOVERY_WORK_IN_PROGRESS15) {
      if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().EMERGENCY_VEHICLES)
        sub_cause_code_type = "emergency vehicles";
      else if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().RESCUE_HELICOPTER_LANDING)
        sub_cause_code_type = "rescue helicopter landing";
      else if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().POLICE_ACTIVITY_ONGOING)
        sub_cause_code_type = "police activity ongoing";
      else if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().MEDICAL_EMERGENCY_ONGOING)
        sub_cause_code_type = "medical emergency ongoing";
      else if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().CHILD_ABDUCTION_IN_PROGRESS)
        sub_cause_code_type = "child abduction in progress";
      else if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().PRIORITIZED_VEHICLE)
        sub_cause_code_type = "prioritized vehicle";
      else if (sub_cause_code == RescueAndRecoveryWorkInProgressSubCauseCode().RESCUE_AND_RECOVERY_VEHICLE)
        sub_cause_code_type = "rescue and recovery vehicle";
    } else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION17) {
      if (sub_cause_code == AdverseWeatherConditionExtremeWeatherConditionSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == AdverseWeatherConditionExtremeWeatherConditionSubCauseCode().STRONG_WINDS)
        sub_cause_code_type = "strong winds";
      else if (sub_cause_code == AdverseWeatherConditionExtremeWeatherConditionSubCauseCode().DAMAGING_HAIL)
        sub_cause_code_type = "damaging hail";
      else if (sub_cause_code == AdverseWeatherConditionExtremeWeatherConditionSubCauseCode().HURRICANE)
        sub_cause_code_type = "hurricane";
      else if (sub_cause_code == AdverseWeatherConditionExtremeWeatherConditionSubCauseCode().THUNDERSTORM)
        sub_cause_code_type = "thunderstorm";
      else if (sub_cause_code == AdverseWeatherConditionExtremeWeatherConditionSubCauseCode().TORNADO)
        sub_cause_code_type = "tornado";
      else if (sub_cause_code == AdverseWeatherConditionExtremeWeatherConditionSubCauseCode().BLIZZARD)
        sub_cause_code_type = "blizzard";
    } else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_VISIBILITY18) {
      if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().FOG)
        sub_cause_code_type = "fog";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().SMOKE)
        sub_cause_code_type = "smoke";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().HEAVY_SNOWFALL)
        sub_cause_code_type = "heavy snowfall";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().HEAVY_RAIN)
        sub_cause_code_type = "heavy rain";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().HEAVY_HAIL)
        sub_cause_code_type = "heavy hail";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().LOW_SUN_GLARE)
        sub_cause_code_type = "low sun glare";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().SANDSTORMS)
        sub_cause_code_type = "sandstorms";
      else if (sub_cause_code == AdverseWeatherConditionVisibilitySubCauseCode().SWARMS_OF_INSECTS)
        sub_cause_code_type = "swarms of insects";
    } else if (cause_code == CauseCodeChoice().CHOICE_ADVERSE_WEATHER_CONDITION_PRECIPITATION19) {
      if (sub_cause_code == AdverseWeatherConditionPrecipitationSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == AdverseWeatherConditionPrecipitationSubCauseCode().HEAVY_RAIN)
        sub_cause_code_type = "heavy rain";
      else if (sub_cause_code == AdverseWeatherConditionPrecipitationSubCauseCode().HEAVY_SNOWFALL)
        sub_cause_code_type = "heavy snowfall";
      else if (sub_cause_code == AdverseWeatherConditionPrecipitationSubCauseCode().SOFT_HAIL)
        sub_cause_code_type = "soft hail";
    } else if (cause_code == CauseCodeChoice().CHOICE_VIOLENCE20)
      sub_cause_code_type = "not defined";
    else if (cause_code == CauseCodeChoice().CHOICE_SLOW_VEHICLE26) {
      if (sub_cause_code == SlowVehicleSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == SlowVehicleSubCauseCode().MAINTENANCE_VEHICLE)
        sub_cause_code_type = "maintenance vehicle";
      else if (sub_cause_code == SlowVehicleSubCauseCode().VEHICLES_SLOWING_TO_LOOK_AT_ACCIDENT)
        sub_cause_code_type = "vehicles slowing to look at accident";
      else if (sub_cause_code == SlowVehicleSubCauseCode().ABNORMAL_LOAD)
        sub_cause_code_type = "abnormal load";
      else if (sub_cause_code == SlowVehicleSubCauseCode().ABNORMAL_WIDE_LOAD)
        sub_cause_code_type = "abnormal wide load";
      else if (sub_cause_code == SlowVehicleSubCauseCode().CONVOY)
        sub_cause_code_type = "convoy";
      else if (sub_cause_code == SlowVehicleSubCauseCode().SNOWPLOUGH)
        sub_cause_code_type = "snowplough";
      else if (sub_cause_code == SlowVehicleSubCauseCode().DEICING)
        sub_cause_code_type = "deicing";
      else if (sub_cause_code == SlowVehicleSubCauseCode().SALTING_VEHICLES)
        sub_cause_code_type = "salting vehicles";
    } else if (cause_code == CauseCodeChoice().CHOICE_DANGEROUS_END_OF_QUEUE27) {
      if (sub_cause_code == DangerousEndOfQueueSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == DangerousEndOfQueueSubCauseCode().SUDDEN_END_OF_QUEUE)
        sub_cause_code_type = "sudden end of queue";
      else if (sub_cause_code == DangerousEndOfQueueSubCauseCode().QUEUE_OVER_HILL)
        sub_cause_code_type = "queue over hill";
      else if (sub_cause_code == DangerousEndOfQueueSubCauseCode().QUEUE_AROUND_BEND)
        sub_cause_code_type = "queue around bend";
      else if (sub_cause_code == DangerousEndOfQueueSubCauseCode().QUEUE_IN_TUNNEL)
        sub_cause_code_type = "queue in tunnel";
    } else if (cause_code == CauseCodeChoice().CHOICE_PUBLIC_TRANSPORT_VEHICLE_APPROACHING28)
      sub_cause_code_type = "not defined";
    else if (cause_code == CauseCodeChoice().CHOICE_VEHICLE_BREAKDOWN91) {
      if (sub_cause_code == VehicleBreakdownSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().LACK_OF_FUEL)
        sub_cause_code_type = "lack of fuel";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().LACK_OF_BATTERY_POWER)
        sub_cause_code_type = "lack of battery power";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().ENGINE_PROBLEM)
        sub_cause_code_type = "engine problem";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().TRANSMISSION_PROBLEM)
        sub_cause_code_type = "transmission problem";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().ENGINE_COOLING_PROBLEM)
        sub_cause_code_type = "engine cooling problem";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().BRAKING_SYSTEM_PROBLEM)
        sub_cause_code_type = "braking system problem";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().STEERING_PROBLEM)
        sub_cause_code_type = "steering problem";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().TYRE_PUNCTURE)
        sub_cause_code_type = "tyre puncture";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().TYRE_PRESSURE_PROBLEM)
        sub_cause_code_type = "tyre pressure problem";
      else if (sub_cause_code == VehicleBreakdownSubCauseCode().VEHICLE_ON_FIRE)
        sub_cause_code_type = "vehicle on fire";
    } else if (cause_code == CauseCodeChoice().CHOICE_POST_CRASH92) {
      if (sub_cause_code == PostCrashSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == PostCrashSubCauseCode().ACCIDENT_WITHOUT_E_CALL_TRIGGERED)
        sub_cause_code_type = "accident without e-Call triggered";
      else if (sub_cause_code == PostCrashSubCauseCode().ACCIDENT_WITH_E_CALL_MANUALLY_TRIGGERED)
        sub_cause_code_type = "accident with e-Call manually triggered";
      else if (sub_cause_code == PostCrashSubCauseCode().ACCIDENT_WITH_E_CALL_AUTOMATICALLY_TRIGGERED)
        sub_cause_code_type = "accident with e-Call automatically triggered";
      else if (sub_cause_code == PostCrashSubCauseCode().ACCIDENT_WITH_E_CALL_TRIGGERED_WITHOUT_ACCESS_TO_CELLULAR_NETWORK)
        sub_cause_code_type = "accident with e-Call triggered without a possible access to a cell network";
    } else if (cause_code == CauseCodeChoice().CHOICE_HUMAN_PROBLEM93) {
      if (sub_cause_code == HumanProblemSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == HumanProblemSubCauseCode().GLYCEMIA_PROBLEM)
        sub_cause_code_type = "glycemia problem";
      else if (sub_cause_code == HumanProblemSubCauseCode().HEART_PROBLEM)
        sub_cause_code_type = "heart problem";
    } else if (cause_code == CauseCodeChoice().CHOICE_STATIONARY_VEHICLE94) {
      if (sub_cause_code == StationaryVehicleSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == StationaryVehicleSubCauseCode().HUMAN_PROBLEM)
        sub_cause_code_type = "human problem";
      else if (sub_cause_code == StationaryVehicleSubCauseCode().VEHICLE_BREAKDOWN)
        sub_cause_code_type = "vehicle breakdown";
      else if (sub_cause_code == StationaryVehicleSubCauseCode().POST_CRASH)
        sub_cause_code_type = "post crash";
      else if (sub_cause_code == StationaryVehicleSubCauseCode().PUBLIC_TRANSPORT_STOP)
        sub_cause_code_type = "public transport stop";
      else if (sub_cause_code == StationaryVehicleSubCauseCode().CARRYING_DANGEROUS_GOODS)
        sub_cause_code_type = "carrying dangerous goods";
      else if (sub_cause_code == StationaryVehicleSubCauseCode().VEHICLE_ON_FIRE)
        sub_cause_code_type = "vehicle on fire";
    } else if (cause_code == CauseCodeChoice().CHOICE_EMERGENCY_VEHICLE_APPROACHING95) {
      if (sub_cause_code == EmergencyVehicleApproachingSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == EmergencyVehicleApproachingSubCauseCode().EMERGENCY_VEHICLE_APPROACHING)
        sub_cause_code_type = "emergency vehicle approaching";
      else if (sub_cause_code == EmergencyVehicleApproachingSubCauseCode().PRIORITIZED_VEHICLE_APPROACHING)
        sub_cause_code_type = "prioritized vehicle approaching";
    } else if (cause_code == CauseCodeChoice().CHOICE_HAZARDOUS_LOCATION_DANGEROUS_CURVE96) {
      if (sub_cause_code == HazardousLocationDangerousCurveSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == HazardousLocationDangerousCurveSubCauseCode().DANGEROUS_LEFT_TURN_CURVE)
        sub_cause_code_type = "dangerous left turn curve";
      else if (sub_cause_code == HazardousLocationDangerousCurveSubCauseCode().DANGEROUS_RIGHT_TURN_CURVE)
        sub_cause_code_type = "dangerous right turn curve";
      else if (sub_cause_code == HazardousLocationDangerousCurveSubCauseCode().MULTIPLE_CURVES_STARTING_WITH_UNKNOWN_TURNING_DIRECTION)
        sub_cause_code_type = "multiple curves starting with unknown turning direction";
      else if (sub_cause_code == HazardousLocationDangerousCurveSubCauseCode().MULTIPLE_CURVES_STARTING_WITH_LEFT_TURN)
        sub_cause_code_type = "multiple curves starting with left turn";
      else if (sub_cause_code == HazardousLocationDangerousCurveSubCauseCode().MULTIPLE_CURVES_STARTING_WITH_RIGHT_TURN)
        sub_cause_code_type = "multiple curves starting with right turn";
    } else if (cause_code == CauseCodeChoice().CHOICE_COLLISION_RISK97) {
      if (sub_cause_code == CollisionRiskSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == CollisionRiskSubCauseCode().LONGITUDINAL_COLLISION_RISK)
        sub_cause_code_type = "longitudinal collision risk";
      else if (sub_cause_code == CollisionRiskSubCauseCode().CROSSING_COLLISION_RISK)
        sub_cause_code_type = "crossing collision risk";
      else if (sub_cause_code == CollisionRiskSubCauseCode().LATERAL_COLLISION_RISK)
        sub_cause_code_type = "lateral collision risk";
      else if (sub_cause_code == CollisionRiskSubCauseCode().VULNERABLE_ROAD_USER)
        sub_cause_code_type = "collision risk involving vulnerable road user";
      else if (sub_cause_code == CollisionRiskSubCauseCode().COLLISION_RISK_WITH_PEDESTRIAN)
        sub_cause_code_type = "collision risk involving pedestrian";
      else if (sub_cause_code == CollisionRiskSubCauseCode().COLLISION_RISK_WITH_CYCLIST)
        sub_cause_code_type = "collision risk involving cyclist";
      else if (sub_cause_code == CollisionRiskSubCauseCode().COLLISION_RISK_WITH_MOTOR_VEHICLE)
        sub_cause_code_type = "collision risk involving motor vehicle";
    } else if (cause_code == CauseCodeChoice().CHOICE_SIGNAL_VIOLATION98) {
      if (sub_cause_code == SignalViolationSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == SignalViolationSubCauseCode().STOP_SIGN_VIOLATION)
        sub_cause_code_type = "stop sign violation";
      else if (sub_cause_code == SignalViolationSubCauseCode().TRAFFIC_LIGHT_VIOLATION)
        sub_cause_code_type = "traffic light violation";
      else if (sub_cause_code == SignalViolationSubCauseCode().TURNING_REGULATION_VIOLATION)
        sub_cause_code_type = "turning regulation violation";
    } else if (cause_code == CauseCodeChoice().CHOICE_DANGEROUS_SITUATION99) {
      if (sub_cause_code == DangerousSituationSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == DangerousSituationSubCauseCode().EMERGENCY_ELECTRONIC_BRAKE_ENGAGED)
        sub_cause_code_type = "emergency electronic brake engaged";
      else if (sub_cause_code == DangerousSituationSubCauseCode().PRE_CRASH_SYSTEM_ENGAGED)
        sub_cause_code_type = "pre-crash system engaged";
      else if (sub_cause_code == DangerousSituationSubCauseCode().ESP_ENGAGED)
        sub_cause_code_type = "ESP (electronic stability program) engaged";
      else if (sub_cause_code == DangerousSituationSubCauseCode().ABS_ENGAGED)
        sub_cause_code_type = "ABS (anti-lock braking system) engaged";
      else if (sub_cause_code == DangerousSituationSubCauseCode().EB_ENGAGED)
        sub_cause_code_type = "AEB (automatic emergency braking) engaged";
      else if (sub_cause_code == DangerousSituationSubCauseCode().BRAKE_WARNING_ENGAGED)
        sub_cause_code_type = "brake warning engaged";
      else if (sub_cause_code == DangerousSituationSubCauseCode().COLLISION_RISK_WARNING_ENGAGED)
        sub_cause_code_type = "collision risk warning engaged";
    } else if (cause_code == CauseCodeChoice().CHOICE_RAILWAY_LEVEL_CROSSING100) {
      if (sub_cause_code == RailwayLevelCrossingSubCauseCode().UNAVAILABLE)
        sub_cause_code_type = "unavailable";
      else if (sub_cause_code == RailwayLevelCrossingSubCauseCode().DO_NOT_CROSS_ABNORMAL_SITUATION)
        sub_cause_code_type = "do not cross abnormal situation";
      else if (sub_cause_code == RailwayLevelCrossingSubCauseCode().CLOSED)
        sub_cause_code_type = "closed";
      else if (sub_cause_code == RailwayLevelCrossingSubCauseCode().UNGUARDED)
        sub_cause_code_type = "unguarded";
      else if (sub_cause_code == RailwayLevelCrossingSubCauseCode().NOMINAL)
        sub_cause_code_type = "nominal";
    }
    return sub_cause_code_type;
  } else {
    throw std::invalid_argument("SituationContainer is not present!");
  }
}

}  // namespace etsi_its_denm_ts_msgs::access
