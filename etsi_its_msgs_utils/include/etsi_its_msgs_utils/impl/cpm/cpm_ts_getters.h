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
  inline uint32_t getStationID(const CollectivePerceptionMessage& cpm){
    return getStationID(cpm.header);
  }

  /**
   * @brief Get the Reference Time object
   * 
   * @param cpm CPM to get the ReferenceTime-Value from
   * @return TimestampIts 
   */
  inline TimestampIts getReferenceTime(const CollectivePerceptionMessage& cpm){
    return cpm.payload.management_container.reference_time;
  }

  /**
   * @brief Get the ReferenceTime-Value
   * 
   * @param cpm CPM to get the ReferenceTime-Value from 
   * @return uint64_t the ReferenceTime-Value
   */
  inline uint64_t getReferenceTimeValue(const CollectivePerceptionMessage& cpm){
    return getReferenceTime(cpm).value;
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
  inline gm::PointStamped getUTMPosition(const CollectivePerceptionMessage& cpm, int& zone, bool& northp){
    return getUTMPosition(cpm.payload.management_container.reference_position, zone, northp);
  }
  
} // namespace etsi_its_cpm_ts_msgs::access
