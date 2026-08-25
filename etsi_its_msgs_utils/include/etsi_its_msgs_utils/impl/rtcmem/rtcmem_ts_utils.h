/*
=============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2026 Virtual Vehicle Research GmbH

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
 * @file impl/rtcmem/rtcmem_ts_utils.h
 * @brief Utility functions for the ETSI ITS RTCMEM
 */

#include <ctime>
#include <limits>

#pragma once

namespace etsi_its_rtcmem_ts_msgs {

namespace access {

static constexpr int HOUR_IN_SECONDS = 3600;
static constexpr int64_t FACTOR_SECONDS_TO_NANOSECONDS = 1000000000LL;

// ETSI Timemark value equals 0.1 seconds and 10e8 nanoseconds
static constexpr int64_t FACTOR_ETSI_TIMEMARK_TO_NANOSECONDS = 100000000LL;
static constexpr float FACTOR_ETSI_TIMEMARK_TO_SECONDS = 0.1f;

/**
 * @brief Get the unix seconds of the beginning of a year that corresponds to a given unix timestamp
 *
 * @param unixSecond timestamp that defines the year for that the unix seconds for the beginning of the year should be derived
 * @return uint64_t unix seconds of the beginning of the year
 */
inline uint64_t getUnixSecondsOfYear(const uint64_t unixSecond) {
  time_t ts = static_cast<time_t>(unixSecond);
  struct tm* timeinfo = gmtime(&ts);

  timeinfo->tm_sec = 0;
  timeinfo->tm_min = 0;
  timeinfo->tm_hour = 0;
  timeinfo->tm_mday = 1;
  timeinfo->tm_mon = 0;

  return timegm(timeinfo);
}

/**
 * @brief Get the unix nanoseconds from a MinuteOfTheYear object
 *
 * @param moy given MinuteOfTheYear object
 * @param unix_nanoseconds_estimate unix timestamp to derive the current year from in nanoseconds
 * @return uint64_t unix timestamp according to the given MinuteOfTheYear in nanoseconds
 */
inline uint64_t getUnixNanosecondsFromMinuteOfTheYear(const MinuteOfTheYear& moy,
                                                      const uint64_t unix_nanoseconds_estimate) {
  const uint64_t unix_seconds_of_year =
      getUnixSecondsOfYear(static_cast<uint64_t>(unix_nanoseconds_estimate * 1e-9));
  return (unix_seconds_of_year + static_cast<uint64_t>(moy.value) * 60ULL) * 1000000000ULL;
}

/**
 * @brief Get the unix nanoseconds from a RTCMcorrections object
 *
 * @param rtcm given RTCMcorrections object
 * @param unix_nanoseconds_estimate unix timestamp to derive the current year from in nanoseconds
 * @return uint64_t unix timestamp according to the stored MinuteOfTheYear in nanoseconds
 */
inline uint64_t getUnixNanosecondsFromRTCMcorrections(const RTCMcorrections& rtcm,
                                                      const uint64_t unix_nanoseconds_estimate) {
  return getUnixNanosecondsFromMinuteOfTheYear(getMinuteOfTheYear(rtcm), unix_nanoseconds_estimate);
}

/**
 * @brief Get the unix nanoseconds from a RTCMEM object
 *
 * @param rtcmem given RTCMEM object
 * @param unix_nanoseconds_estimate unix timestamp to derive the current year from in nanoseconds
 * @return uint64_t unix timestamp according to the stored MinuteOfTheYear in nanoseconds
 */
inline uint64_t getUnixNanoseconds(const RTCMEM& rtcmem, const uint64_t unix_nanoseconds_estimate) {
  return getUnixNanosecondsFromRTCMcorrections(rtcmem.rtcmc, unix_nanoseconds_estimate);
}

/**
 * @brief Convert Latitude integer representation into degrees
 *
 * Value is encoded in 1e-7 degrees.
 * If UNAVAILABLE, returns NaN.
 *
 * @param lat Latitude object to interpret
 * @return double latitude in degrees or NaN if unavailable
 */
inline double interpretLatitudeInDegrees(const Latitude& lat) {
  if (lat.value == Latitude::UNAVAILABLE) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(lat.value) * 1e-7;
}

/**
 * @brief Convert Longitude integer representation into degrees
 *
 * Value is encoded in 1e-7 degrees.
 * If UNAVAILABLE, returns NaN.
 *
 * @param lon Longitude object to interpret
 * @return double longitude in degrees or NaN if unavailable
 */
inline double interpretLongitudeInDegrees(const Longitude& lon) {
  if (lon.value == Longitude::UNAVAILABLE) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(lon.value) * 1e-7;
}

} // namespace access

} // namespace etsi_its_rtcmem_ts_msgs
