/*
=============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

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
 * @brief Getter functions for the ETSI ITS DENM
 */

#pragma once

namespace cdd = etsi_its_msgs::cdd_access;
namespace etsi_its_denm_msgs {

namespace access {


  /**
   * @brief Get the Station ID object
   *
   * @param denm DENM to get the StationID value from
   * @return stationID value
   */
  inline uint32_t getStationID(const DENM& denm){
    return cdd::getStationID(denm.header);
  }

  /**
   * @brief 
   * 
   * @param denm DENM to get the UTM Position from
   * @param zone the UTM zone (zero means UPS)
   * @param northp hemisphere (true means north, false means south)
   * @return gm::PointStamped geometry_msgs::PointStamped of the given position
   */
  inline gm::PointStamped getUTMPosition(const DENM& denm, int& zone, bool& northp){
    return cdd::getUTMPosition(denm.denm.management.event_position, zone, northp);
  }

  /**
   * @brief Get the stationType object
   * 
   * @param denm DENM to get the stationType value from
   * @return stationType value
   */
  inline uint8_t getStationType(const DENM& denm){
    return denm.denm.management.station_type.value;
  }

  /**
   * @brief Get the Cause Code object
   * 
   * @param denm DENM to get the causeCode value from
   * @return causeCode value
   */
  inline int getCauseCode(const DENM& denm){
    return denm.denm.situation.event_type.cause_code.value;
  }

  /**
   * @brief Get the Sub Cause Code object
   * 
   * @param denm DENM to get the subCauseCode value from
   * @return subCauseCode value
   */
  inline int getSubCauseCode(const DENM& denm){
    return denm.denm.situation.event_type.sub_cause_code.value;
  }

  /**
   * @brief Get the Cause Code Type object
   * 
   * @param denm DENM to get the causeCodeType value from
   * @return causeCodeType value
   */
  inline std::string getCauseCodeType(const DENM& denm){
    int cause_code = getCauseCode(denm);
    std::string cause_code_type;

    if(cause_code == 1) cause_code_type = "traffic condition";
    else if(cause_code == 2) cause_code_type = "accident";
    else if(cause_code == 3) cause_code_type = "roadworks";
    else if(cause_code == 5) cause_code_type = "impassibility";
    else if(cause_code == 6) cause_code_type = "adverse weather condition - adhesion";
    else if(cause_code == 7) cause_code_type = "aquaplanning";
    else if(cause_code == 9) cause_code_type = "hazardous location - surface condition";
    else if(cause_code == 10) cause_code_type = "hazardous location - obstacle on the road";
    else if(cause_code == 11) cause_code_type = "hazardous location - animal on the road";
    else if(cause_code == 12) cause_code_type = "human presence on the road";
    else if(cause_code == 14) cause_code_type = "wrong way driving";
    else if(cause_code == 15) cause_code_type = "rescue and recovery in progress";
    else if(cause_code == 17) cause_code_type = "adverse weather condition - extreme weather condition";
    else if(cause_code == 18) cause_code_type = "adverse weather condition - visibility";
    else if(cause_code == 19) cause_code_type = "adverse weather condition - precipitation";
    else if(cause_code == 26) cause_code_type = "slow vehicle";
    else if(cause_code == 27) cause_code_type = "dangerous end of queue";
    else if(cause_code == 91) cause_code_type = "vehicle breakdown";
    else if(cause_code == 92) cause_code_type = "post crash";
    else if(cause_code == 93) cause_code_type = "human problem";
    else if(cause_code == 94) cause_code_type = "stationary vehicle";
    else if(cause_code == 95) cause_code_type = "emergency vehicle approaching";
    else if(cause_code == 96) cause_code_type = "hazardous location - dangerous curve";
    else if(cause_code == 97) cause_code_type = "collision risk";
    else if(cause_code == 98) cause_code_type = "signal violation";
    else if(cause_code == 99) cause_code_type = "dangerous situation";

    return cause_code_type;
  }

  //definition of cause codes and sub cause codes
  //DENM-Documentation: https://www.etsi.org/deliver/etsi_en/302600_302699/30263703/01.02.01_30/en_30263703v010201v.pdf

  /**
   * @brief Get the Sub Cause Code Type object
   * 
   * @param denm DENM to get the subCauseCodeType value from
   * @return causeCodeType value 
   */
  inline std::string getSubCauseCodeType(const DENM& denm){
    int cause_code = getCauseCode(denm);
    int sub_cause_code = getSubCauseCode(denm);
    std::string sub_cause_code_type;
    if(cause_code == 1) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "not defined";
      else if(sub_cause_code == 2) sub_cause_code_type = "traffic jam slowly increasing";
      else if(sub_cause_code == 3) sub_cause_code_type = "traffic jam increasing";
      else if(sub_cause_code == 4) sub_cause_code_type = "traffic jam strongly increasing";
      else if(sub_cause_code == 5) sub_cause_code_type = "traffic stationary";
      else if(sub_cause_code == 6) sub_cause_code_type = "traffic jam slightly decreasing";
      else if(sub_cause_code == 7) sub_cause_code_type = "traffic jam decreasing";
      else if(sub_cause_code == 8) sub_cause_code_type = "traffic jam strongly decreasing";
      }
    else if(cause_code == 2) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 7) sub_cause_code_type = "not defined";
      else if(sub_cause_code == 8) sub_cause_code_type = "assistance requested (e-Call)";
      }
    else if(cause_code == 3) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 3) sub_cause_code_type = "not defined";
      else if(sub_cause_code == 4) sub_cause_code_type = "short-term stationary roadworks";
      else if(sub_cause_code == 5) sub_cause_code_type = "street cleaning";
      else if(sub_cause_code == 6) sub_cause_code_type = "winter service";
    }
    else if(cause_code == 5) sub_cause_code_type = "not defined";
    else if(cause_code == 6) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 10) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 7) sub_cause_code_type = "not defined";
    else if(cause_code == 9) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 9) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 10) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 7) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 11) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 4) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 12) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 3) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 14) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "vehicle driving in wrong lane";
      else if(sub_cause_code == 2) sub_cause_code_type = "vehicle driving in wrong driving direction";
      }
    else if(cause_code == 15) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 5) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 17) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 6) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 18) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 8) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 19) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 3) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 26) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 8) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 27) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code >= 1 && sub_cause_code <= 8) sub_cause_code_type = "not defined";
      }
    else if(cause_code == 91) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "lack of fuel";
      else if(sub_cause_code == 2) sub_cause_code_type = "lack of battery";
      else if(sub_cause_code == 3) sub_cause_code_type = "engine problem";
      else if(sub_cause_code == 4) sub_cause_code_type = "transmission problem";
      else if(sub_cause_code == 5) sub_cause_code_type = "engine cooling problem";
      else if(sub_cause_code == 6) sub_cause_code_type = "braking system problem";
      else if(sub_cause_code == 7) sub_cause_code_type = "steering problem";
      else if(sub_cause_code == 8) sub_cause_code_type = "tyre puncture";
      }
    else if(cause_code == 92) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "accident without e-Call triggered";
      else if(sub_cause_code == 2) sub_cause_code_type = "accident with e-Call manually triggered";
      else if(sub_cause_code == 3) sub_cause_code_type = "accident with e-Call automatical triggered";
      else if(sub_cause_code == 4) sub_cause_code_type = "accident with e-Call triggered without a possible access to a cell network";
      }
    else if(cause_code == 93) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "glycaemia problem";
      else if(sub_cause_code == 2) sub_cause_code_type = "heart problem";
      }
    else if(cause_code == 94) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "human problem";
      else if(sub_cause_code == 2) sub_cause_code_type = "vehicle breakdown";
      else if(sub_cause_code == 3) sub_cause_code_type = "post crash";
      else if(sub_cause_code == 4) sub_cause_code_type = "public transport stop";
      else if(sub_cause_code == 5) sub_cause_code_type = "carrying dangerous goods";
      }
    else if(cause_code == 95) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "emergency vehicle approaching";
      else if(sub_cause_code == 2) sub_cause_code_type = "prioritized vehicle approaching";
      }
    else if(cause_code == 96) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "dangerous left turn curve";
      else if(sub_cause_code == 2) sub_cause_code_type = "dangerous right turn curve";
      else if(sub_cause_code == 3) sub_cause_code_type = "multiple curves starting with unknown turning direction";
      else if(sub_cause_code == 4) sub_cause_code_type = "multiple curves starting with left turn";
      else if(sub_cause_code == 5) sub_cause_code_type = "multiple curves starting with right turn";
      }
    else if(cause_code == 97) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "longitudinal collision risk";
      else if(sub_cause_code == 2) sub_cause_code_type = "crossing collision risk";
      else if(sub_cause_code == 3) sub_cause_code_type = "lateral collision risk";
      else if(sub_cause_code == 4) sub_cause_code_type = "collision risk involving vulnerable road user";
      }
    else if(cause_code == 98) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "stop sign violation";
      else if(sub_cause_code == 2) sub_cause_code_type = "traffic light violation";
      else if(sub_cause_code == 3) sub_cause_code_type = "turning regulation violation";
      }
    else if(cause_code == 99) {
      if(sub_cause_code == 0) sub_cause_code_type = "unavailable";
      else if(sub_cause_code == 1) sub_cause_code_type = "emergency electronic break lights";
      else if(sub_cause_code == 2) sub_cause_code_type = "pre-crash system activated";
      else if(sub_cause_code == 3) sub_cause_code_type = "ESP (electronic stability program) activated";
      else if(sub_cause_code == 4) sub_cause_code_type = "ABS (anti-lock breaking system) activated";
      else if(sub_cause_code == 5) sub_cause_code_type = "AEB (automatic emergency breaking) activated";
      else if(sub_cause_code == 6) sub_cause_code_type = "break warning activated";
      else if(sub_cause_code == 7) sub_cause_code_type = "collision risk warning activated";
      }
      return sub_cause_code_type;
  }

  /**
   * @brief Get the Heading object
   * 
   * @param denm DENM to get the Heading-Value from
   * @return getHeading value
   */
  inline double getHeading(const DENM& denm){
    return cdd::getHeading(denm.denm.location.event_position_heading);
  }
  
  /**
   * @brief Get the IsHeadingPresent object
   * 
   * @param denm DENM to get the IsHeadingPresent-Value from
   * @return IsHeadingPresent-Value (true or false)
   */
  inline bool getIsHeadingPresent(const DENM& denm){
    return denm.denm.location.event_position_heading_is_present;
  }

  /**
   * @brief Get the vehicle speed
   * 
   * @param cam CAM to get the speed value from
   * @return speed value in m/s as decimal number
   */
  inline double getSpeed(const DENM& denm){
    return cdd::getSpeed(denm.denm.location.event_speed);
  }

  /**
   * @brief Get the IsSpeedPresent object
   * 
   * @param denm DENM to get the IsSpeedPresent-Value from
   * @return IsSpeedPresent-Value (true or false)
   */
  inline bool getIsSpeedPresent(const DENM& denm){
    return denm.denm.location.event_speed_is_present;
  }

  /**
   * @brief Get the Reference Time object
   * 
   * @param denm DENM to get the ReferenceTime-Value from
   * @return TimestampIts 
   */
  inline TimestampIts getReferenceTime(const DENM& denm){
    return denm.denm.management.reference_time;
  }
} // namespace access

} // namespace etsi_its_denm_msgs
