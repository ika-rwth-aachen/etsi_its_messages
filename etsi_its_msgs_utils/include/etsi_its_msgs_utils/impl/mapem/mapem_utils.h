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
 * @file impl/mapem/mapem_utils.h
 * @brief Utility functions for the ETSI ITS MAPEM
 */

#include <etsi_its_msgs_utils/impl/constants.h>

#pragma once

namespace J2735 = etsi_its_msgs::J2735_access;
namespace etsi_its_mapem_msgs {

namespace access {

  /**
   * @brief Get the unix nanoseconds from MinuteOfTheYear object
   * 
   * @param moy given MinuteOfTheYear object
   * @param unix_timestamp_estimate unix timestamp to derive the current year from
   * @return uint64_t unix timestamp according to the given MinuteOfTheYear
   */
  inline uint64_t getUnixNanosecondsFromMinuteOfTheYear(const MAPEM& mapem, const uint64_t unix_timestamp_estimate)
  {
    return J2735::getUnixNanosecondsFromMinuteOfTheYear(mapem.map.time_stamp, unix_timestamp_estimate);
  }

} // namespace etsi_its_mapem_msgs
} // namespace access
