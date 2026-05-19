// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/cpm/cpm_ts_utils.h
 * @brief Utility functions for the ETSI ITS CPM (TS)
 */

#pragma once

/**
 * @brief Get the Unix-Nanoseconds from a given ReferenceTime object
 *
 * @param reference_time the ReferenceTime object to get the Unix-Nanoseconds from
 * @param n_leap_seconds number of leap-seconds since 2004. (Default: etsi_its_msgs::N_LEAP_SECONDS)
 * @return uint64_t the corresponding Unix-Nanoseconds
 */
inline uint64_t getUnixNanosecondsFromReferenceTime(const TimestampIts& reference_time) {
  double unix_time_with_leap_seconds = reference_time.value * 1e-3 + etsi_its_msgs::UNIX_SECONDS_2004;
  uint16_t n_leap_seconds =
      etsi_its_msgs::getLeapSecondInsertionsSince2004(static_cast<uint64_t>(unix_time_with_leap_seconds));
  return (unix_time_with_leap_seconds - n_leap_seconds) * 1e9;
}