/**
 * @file
 * @brief Utility functions for etsi_its_cam_msgs CAM
 */

#include <etsi_its_msgs/impl/cdd/cdd_checks.h>
#include <etsi_its_msgs/impl/constants.h>

#pragma once

namespace cdd = etsi_its_msgs::cdd_access;
namespace etsi_its_cam_msgs {

namespace access {

  /**
   * @brief Get the TimestampITS from a given GenerationDeltaTime object
   * 
   * @param generation_delta_time the GenerationDeltaTime object to get the TimestampITS from
   * @param timestamp_estimate estimated time to calculate the corresponding generation from
   * @return TimestampIts the corresponding TimestampITS object
   */
  inline TimestampIts getTimestampITSFromGenerationDeltaTime(const GenerationDeltaTime& generation_delta_time, const TimestampIts& timestamp_estimate)
  {
    TimestampIts t_its;
    t_its.value = std::floor(timestamp_estimate.value/65536)*65536+generation_delta_time.value;
    cdd::throwIfOutOfRange(t_its.value, TimestampIts::MIN, TimestampIts::MAX, "TimestampIts");
    return t_its;
  }

  /**
   * @brief Get the Unix-Nanoseconds from a given GenerationDeltaTime object
   * 
   * @param generation_delta_time the GenerationDeltaTime object to get the Unix-Nanoseconds from
   * @param timestamp_estimate estimated time to calculate the corresponding generation from
   * @param n_leap_seconds number of leap-seconds since 2004. (Default: etsi_its_msgs::N_LEAP_SECONDS)
   * @return uint64_t the corresponding Unix-Nanoseconds
   */
  inline uint64_t getUnixNanosecondsFromGenerationDeltaTime(const GenerationDeltaTime& generation_delta_time, const TimestampIts& timestamp_estimate, const uint16_t n_leap_seconds = etsi_its_msgs::N_LEAP_SECONDS)
  {
    TimestampIts t_its = getTimestampITSFromGenerationDeltaTime(generation_delta_time, timestamp_estimate);
    return t_its.value*1e6+etsi_its_msgs::UNIX_SECONDS_2004*1e9-n_leap_seconds*1e9;
  }

  /**
   * @brief Get the Unix Nanoseconds from Generation Delta Time object
   * 
   * @param generation_delta_time the GenerationDeltaTime object to get the Unix-Nanoseconds from
   * @param unix_timestamp_estimate estimated unix-time (in Nanoseconds) to calculate the corresponding generation from
   * @return uint64_t the corresponding Unix-Nanoseconds
   */
  inline uint64_t getUnixNanosecondsFromGenerationDeltaTime(const GenerationDeltaTime& generation_delta_time, const uint64_t unix_timestamp_estimate, const uint16_t n_leap_seconds = etsi_its_msgs::N_LEAP_SECONDS)
  {
    TimestampIts t_its;
    cdd::setTimestampITS(t_its, unix_timestamp_estimate, n_leap_seconds);
    return getUnixNanosecondsFromGenerationDeltaTime(generation_delta_time, t_its, n_leap_seconds);
  }

} // namespace etsi_its_cam_msgs
} // namespace access
