/**
 * @file 
 * @brief Utility functions for all cdd-access functions.
 */

// #include <chrono>
// #include <ctime>

#pragma once

namespace etsi_its_msgs {

namespace cdd_access {

/**
 * @brief Get the Unix-Seconds for 2004-01-01T00:00:00.000Z
 * 
 * @return uint64_t Unix-Seconds for 2004-01-01T00:00:00.000Z
 */
inline uint64_t getUnixSecondsFor2004()
{
  // // Create a tm struct to represent the date 2004-01-01
  // std::tm timeInfo;
  // timeInfo.tm_year = 2004 - 1900; // Years since 1900
  // timeInfo.tm_mon = 0; // January is month 0
  // timeInfo.tm_mday = 1; // 1st day of the month
  // timeInfo.tm_hour = 0;
  // timeInfo.tm_min = 0;
  // timeInfo.tm_sec = 0;

  // // Convert the tm struct to a time_t value
  // return std::mktime(&timeInfo);

  // 2004-01-01T00:00:00.000Z
  return 1072915200;
}

// inline uint16_t getUTCLeapSecondsSince2004()//const std::chrono::time_point<std::chrono::utc_clock>& utc_time_to)
// {
//   // ToDo
//   // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=104167
//   std::chrono::utc_time utc_2004 = std::chrono::utc_clock::from_sys(getUnixSecondsSince2004());
//   std::chrono::leap_second_info leap_second_info_2004 = get_leap_second_info(utc_2004);
//   std::chrono::leap_second_info leap_second_info_to = get_leap_second_info(utc_time_to);
//   return (leap_second_info_to.elapsed-leap_second_info_2004.elapsed);
// }

} // namespace cdd_access
} // namespace etsi_its_msgs