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
 * @file impl/denm/denm_utils.h
 * @brief Utility functions for the ETSI ITS DENM
 */

#include <etsi_its_msgs_utils/impl/cdd/cdd_checks.h>
#include <etsi_its_msgs_utils/impl/constants.h>

#pragma once

namespace cdd = etsi_its_msgs::cdd_access;
namespace etsi_its_denm_msgs {

namespace access {
  /**
   * @brief Get the Unix-Nanoseconds from a given ReferenceTime object
   *
   * @param reference_time the ReferenceTime object to get the Unix-Nanoseconds from
   * @param n_leap_seconds number of leap-seconds since 2004. (Default: etsi_its_msgs::N_LEAP_SECONDS)
   * @return uint64_t the corresponding Unix-Nanoseconds
   */
  inline uint64_t getUnixNanosecondsFromReferenceTime(const TimestampIts& reference_time, const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second)
  {
    return reference_time.value*1e6+etsi_its_msgs::UNIX_SECONDS_2004*1e9-n_leap_seconds*1e9;
  }
} // namespace etsi_its_denm_msgs
} // namespace access