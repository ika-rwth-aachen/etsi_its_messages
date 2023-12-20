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
 * @file impl/J2735/j2735_v2x_dict_utils.h
 * @brief Utility functions for the J2735 V2X Communications Message Set Dictionary
 */

#include <ctime>

#include <etsi_its_msgs_utils/impl/constants.h>

#pragma once

namespace J2735 = etsi_its_msgs::J2735_access;
namespace etsi_its_msgs {

namespace J2735_access {

  /**
   * @brief Get the unix seconds of the beginning of a year that corresponds to a given unix timestamp
   * 
   * @param unixSecond timestamp that defines the year for that the unix seconds for the beginning of the year should be derived
   * @return uint64_t unix seconds of the beginning of the year
   */
  uint64_t getUnixSecondsOfYear(const uint64_t unixSecond) {

    // Get current time as a time_point
    time_t ts = static_cast<time_t>(unixSecond); // Convert uint64_t to time_t

    struct tm* timeinfo;
    timeinfo = localtime(&ts);

    // Set the timeinfo to the beginning of the year
    timeinfo->tm_sec = 0;
    timeinfo->tm_min = 0;
    timeinfo->tm_hour = 0;
    timeinfo->tm_mday = 1;
    timeinfo->tm_mon = 0;

    return mktime(timeinfo); // Convert struct tm back to Unix timestamp
  }

  /**
   * @brief Get the unix nanoseconds from MinuteOfTheYear object
   * 
   * @param moy given MinuteOfTheYear object
   * @param unix_timestamp_estimate unix timestamp to derive the current year from
   * @return uint64_t unix timestamp according to the given MinuteOfTheYear
   */
  inline uint64_t getUnixNanosecondsFromMinuteOfTheYear(const MinuteOfTheYear& moy, const uint64_t unix_timestamp_estimate)
  {
    return (uint64_t)(moy.value*60) + getUnixSecondsOfYear(unix_timestamp_estimate);
  }

} // namespace etsi_its_msgs
} // namespace J2735_access
