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
 * @file impl/spatem/spatem_ts_getters.h
 * @brief Getter functions for the ETSI ITS SPATEM
 */

#pragma once

namespace etsi_its_spatem_ts_msgs {

namespace access {

#include <etsi_its_msgs_utils/impl/checks.h>

  /**
   * @brief Get the intersection-id
   * 
   * @param intsct_id intersection-id object to get the value from
   * @return uint16_t id of the intersection
   */
  inline uint16_t getIntersectionID(const IntersectionID& intsct_id) {
    return intsct_id.value;
  }

  /**
   * @brief Get the intersection-id of an IntersectionReferenceID object
   * 
   * @param intsct_ref_id IntersectionReferenceID object
   * @return uint16_t id of the intersection
   */
  inline uint16_t getIntersectionID(const IntersectionReferenceID& intsct_ref_id) {
    return getIntersectionID(intsct_ref_id.id);
  }

  /**
   * @brief Get the intersection-id of an IntersectionState object
   * 
   * @param intsct IntersectionState object
   * @return uint16_t id of the intersection
   */
  inline uint16_t getIntersectionID(const IntersectionState& intsct) {
    return getIntersectionID(intsct.id);
  }

  /**
   * @brief Get the MinuteOfTheYear object from a given IntersectionState object
   * 
   * @param intsct IntersectionState object to get the MinuteOfTheYear from
   * @return MinuteOfTheYear object
   */
  inline MinuteOfTheYear getMinuteOfTheYear(const IntersectionState& intsct) {
    throwIfNotPresent(intsct.moy_is_present, "intsct.moy");
    return intsct.moy;
  }

  /**
   * @brief Get the DSecond object from a given IntersectionState object
   * 
   * @param intsct IntersectionState object to get the DSecond from
   * @return DSecond object
   */
  inline DSecond getDSecond(const IntersectionState& intsct) {
    throwIfNotPresent(intsct.time_stamp_is_present, "intsct.time_stamp");
    return intsct.time_stamp;
  }

  /**
   * @brief Get the value of a DSecond object in seconds
   * 
   * @param dsecond DSecond object to get the value from
   * @return double value of DSecond given in seconds
   */
  inline double getDSecondValue(const DSecond& dsecond) {
    return ((double)dsecond.value)*1e-3;
  }

  /**
   * @brief Get the value of an DSecond object from a given IntersectionState object
   * 
   * @param intsct 
   * @return double 
   */
  inline double getDSecondValue(const IntersectionState& intsct) {
    return getDSecondValue(getDSecond(intsct));
  }

  /**
   * @brief Get the Signal Group-ID of an SignalGroupID object
   * 
   * @param signal_group_id SignalGroupID object to get the id from
   * @return uint8_t the id of the signal group
   */
  inline uint8_t getSignalGroupID(const SignalGroupID& signal_group_id) {
    return signal_group_id.value;
  }

  /**
   * @brief Get the Signal Group-ID of an MovementState object
   * 
   * @param mvmt_state MovementState object to get the id from
   * @return uint8_t the id of the signal group
   */
  inline uint8_t getSignalGroupID(const MovementState& mvmt_state) {
    return getSignalGroupID(mvmt_state.signal_group);
  }

  /**
   * @brief Get the current MovementEvent of a given MovementState object
   * 
   * @param mvmt_event MovementState object to get the MovementEvent from
   * @return MovementEvent object
   */
  inline MovementEvent getCurrentMovementEvent(const MovementState& mvmt_state) {
    if(mvmt_state.state_time_speed.array.size()<=0) {
      throw std::runtime_error("MovementEventList is empty.");
    }
    return mvmt_state.state_time_speed.array[0];
  }

  /**
   * @brief Get the Current MovementPhaseState object of a given MovementState object
   * 
   * @param mvmt_state MovementState object to get the MovementPhaseState from
   * @return MovementPhaseState object
   */
  inline MovementPhaseState getCurrentMovementPhaseState(const MovementState& mvmt_state) {
    return getCurrentMovementEvent(mvmt_state).event_state;
  }

  /**
   * @brief Get the Current MovementPhaseState object of a given MovementState object
   * 
   * @param mvmt_state MovementState object to get the MovementPhaseState from
   * @return MovementPhaseState object
   */
  inline uint8_t getCurrentMovementPhaseStateValue(const MovementState& mvmt_state) {
    return getCurrentMovementPhaseState(mvmt_state).value;
  }

} // namespace access

} // namespace etsi_its_spatem_ts_msgs
