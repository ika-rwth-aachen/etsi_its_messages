// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/mcm/mcm_utils.h
 * @brief Utility functions for the ETSI ITS MCM (TR)
 */

#include <etsi_its_msgs_utils/impl/checks.h>
#include <etsi_its_msgs_utils/impl/constants.h>

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_MCM_MCM_UTILS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_MCM_MCM_UTILS_H

/**
 * @brief Get the Unix Nanoseconds from Generation Delta Time object
 *
 * @param generation_delta_time the GenerationDeltaTime object to get the Unix-Nanoseconds from
 * @param unix_timestamp_estimate estimated unix-time (in Nanoseconds) to calculate the corresponding generation from
 * @param n_leap_seconds number of leap-seconds since 2004. (Defaults to the todays number of leap seconds since 2004.)
 * @return uint64_t the corresponding Unix-Nanoseconds
 */
inline uint64_t getUnixNanosecondsFromGenerationDeltaTime(
    const GenerationDeltaTime& generation_delta_time, const uint64_t unix_timestamp_estimate,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  uint64_t ms_estimate_after_utc_2004 =
      unix_timestamp_estimate * 1e-6 + (uint64_t)(n_leap_seconds * 1e3) - etsi_its_msgs::UNIX_SECONDS_2004 * 1e3;
  uint64_t ms_after_utc_2004 = std::floor(ms_estimate_after_utc_2004 / 65536) * 65536 + generation_delta_time.value;
  return ms_after_utc_2004 * 1e6 + etsi_its_msgs::UNIX_SECONDS_2004 * 1e9 - n_leap_seconds * 1e9;
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_MCM_MCM_UTILS_H