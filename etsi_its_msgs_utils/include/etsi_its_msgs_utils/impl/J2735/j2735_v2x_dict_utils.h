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

#include <chrono>

#include <etsi_its_msgs_utils/impl/constants.h>

#pragma once

namespace J2735 = etsi_its_msgs::J2735_access;
namespace etsi_its_msgs {

namespace J2735_access {

  /**
   * @brief Get the Second Of Year object
   * 
   * @param unixSecond 
   * @return uint64_t 
   */
  uint64_t getSecondOfYear(const uint64_t unixSecond) {
    using namespace std::chrono;

    // Get current time as a time_point
    auto currentTime = system_clock::from_time_t(unixSecond);

    // Get current year
    auto currentYear = year_month_day{year_month_day{currentTime}.year()};

    // Get the start of the year
    auto startOfYear = system_clock::time_point(currentYear);

    // Convert the time_point to Unix timestamp (seconds since epoch)
    uint64_t startOfTheYearUnixSeconds = duration_cast<seconds>(startOfYear.time_since_epoch()).count();

    return startOfTheYearUnixSeconds;
  }

  /**
   * @brief Get the Unix Nanoseconds from MinuteOfTheYear object
   *
   * @param generation_delta_time the GenerationDeltaTime object to get the Unix-Nanoseconds from
   * @param unix_timestamp_estimate estimated unix-time (in Nanoseconds) to calculate the corresponding generation from
   * @return uint64_t the corresponding Unix-Nanoseconds
   */
  inline uint64_t getUnixNanosecondsFromMinuteOfTheYear(const MinuteOfTheYear& moy, const uint64_t unix_timestamp_estimate)
  {
    return (uint64_t)moy.value + getSecondOfYear(unix_timestamp_estimate);
  }

} // namespace etsi_its_msgs
} // namespace J2735_access
