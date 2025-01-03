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
 * @file impl/spatem/spatem_ts_setters.h
 * @brief Setter functions for the ETSI ITS SPATEM
 */

#pragma once


namespace etsi_its_spatem_ts_msgs {

namespace access {

#include <etsi_its_msgs_utils/impl/checks.h>

  
  inline void setIntersectionID(IntersectionID& intsct_id, const uint16_t id) {
    throwIfOutOfRange(id, IntersectionID::MIN, IntersectionID::MAX, "IntersectionID");
    intsct_id.value = id;
  }

  inline void setIntersectionID(IntersectionReferenceID& intsct_ref_id, const uint16_t id) {
    setIntersectionID(intsct_ref_id.id, id);
  }

  inline void setIntersectionID(IntersectionState& intsct, const uint16_t id) {
    setIntersectionID(intsct.id, id);
  }

  inline void setMinuteOfTheYear(MinuteOfTheYear& moy, const uint32_t moy_value) {
    throwIfOutOfRange(moy_value, MinuteOfTheYear::MIN, MinuteOfTheYear::MAX, "MinuteOfTheYear");
    moy.value = moy_value;
  }

  inline void setMinuteOfTheYear(IntersectionState& intsct, const uint32_t moy_value) {
    setMinuteOfTheYear(intsct.moy, moy_value);
    intsct.moy_is_present = true;
  }

  inline void setDSecond(DSecond& dsecond, const uint32_t dsecond_value) {
    throwIfOutOfRange(dsecond_value, DSecond::MIN, DSecond::MAX, "DSecond");
    dsecond.value = dsecond_value;
  }

  inline void setDSecond(DSecond& dsecond, const double dsecond_value) {
    uint32_t dsecond_value_ms = (uint32_t)(dsecond_value*1e3);
    setDSecond(dsecond, dsecond_value_ms);
  }

  inline void setDSecond(IntersectionState& intsct, const uint32_t dsecond_value) {
    setDSecond(intsct.time_stamp, dsecond_value);
    intsct.time_stamp_is_present = true;
  }

  inline void setDSecond(IntersectionState& intsct, const double dsecond_value) {
    setDSecond(intsct.time_stamp, dsecond_value);
    intsct.time_stamp_is_present = true;
  }

  inline void setSignalGroupID(SignalGroupID& signal_group_id, const uint8_t id) {
    throwIfOutOfRange(id, SignalGroupID::MIN, SignalGroupID::MAX, "SignalGroupID");
    signal_group_id.value = id;
  }

  inline void setSignalGroupID(MovementState& movement_state, const uint8_t id) {
    setSignalGroupID(movement_state.signal_group, id);
  }

} // namespace access

} // namespace etsi_its_spatem_ts_msgs
