/*
=============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

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
 * @file impl/spatem/spatem_ts_utils.h
 * @brief Utility functions for the ETSI ITS SPATEM
 */

#include <ctime>

#pragma once

namespace etsi_its_spatem_ts_msgs {

namespace access {

  const std::array<float, 4> color_grey {0.5, 0.5, 0.5, 1.0};
  const std::array<float, 4> color_green {0.18, 0.79, 0.21, 1.0};
  const std::array<float, 4> color_orange {0.9, 0.7, 0.09, 1.0};
  const std::array<float, 4> color_red {0.8, 0.2, 0.2, 1.0};

  enum time_mark_value_interpretation { normal, undefined, over_an_hour, leap_second };

  /**
   * @brief Get the unix seconds of the beginning of a year that corresponds to a given unix timestamp
   * 
   * @param unixSecond timestamp that defines the year for that the unix seconds for the beginning of the year should be derived
   * @return uint64_t unix seconds of the beginning of the year
   */
  inline uint64_t getUnixSecondsOfYear(const uint64_t unixSecond) {

    // Get current time as a time_point
    time_t ts = static_cast<time_t>(unixSecond); // Convert uint64_t to time_t

    struct tm* timeinfo;
    timeinfo = gmtime(&ts);

    // Set the timeinfo to the beginning of the year
    timeinfo->tm_sec = 0;
    timeinfo->tm_min = 0;
    timeinfo->tm_hour = 0;
    timeinfo->tm_mday = 1;
    timeinfo->tm_mon = 0;

    return timegm(timeinfo); // Convert struct tm back to Unix timestamp
  }

  /**
   * @brief Get the unix nanoseconds from MinuteOfTheYear object
   * 
   * @param moy given MinuteOfTheYear object
   * @param unix_nanoseconds_estimate unix timestamp to derive the current year from in nanoseconds
   * @return uint64_t unix timestamp according to the given MinuteOfTheYear in nanoseconds
   */
  inline uint64_t getUnixNanosecondsFromMinuteOfTheYear(const MinuteOfTheYear& moy, const uint64_t unix_nanoseconds_estimate) {
    return ((uint64_t)(moy.value*60) + getUnixSecondsOfYear(unix_nanoseconds_estimate*1e-9))*1e9;
  }

  /**
   * @brief Interprets the TimeIntervalConfidence type as a float value (see etsi definition)
   * 
   * @param encoded_probability Value from msg type TimeIntervalConfidence
   * @return confidence as float value [0, 1]
   */
  inline float interpretTimeIntervalConfidenceAsFloat(const uint16_t encoded_probability) {
    float probability = 0;

    switch (encoded_probability)
    {
      case 0:
        probability = 0.21;
        break;
      case 1:
        probability = 0.36;
        break;
      case 2: 
        probability = 0.47;
        break;
      case 3:
        probability = 0.56;
        break;
      case 4:
        probability = 0.62;
        break;
      case 5: 
        probability = 0.68;
        break;
      case 6:
        probability = 0.73;
        break;
      case 7:
        probability = 0.77;
        break;
      case 8:
        probability = 0.81;
        break;
      case 9:
        probability = 0.85;
        break;
      case 10:
        probability = 0.88;
        break;
      case 11:
        probability = 0.91;
        break;
      case 12:
        probability = 0.94;
        break;
      case 13:
        probability = 0.96;
        break;
      case 14:
        probability = 0.98;
        break; 
      case 15:
        probability = 1.0;
        break;
    }

    return probability;
  }

  /**
   * @brief Interprets the MovementPhaseState type as a color (see etsi definition)
   * 
   * @param value Encoded color value from msg type MovementPhaseState
   * @return 4-dimensional array with color values as follows: r, g, b, a, each of these values within a range between [0, 1]
   */
  inline std::array<float, 4> interpretMovementPhaseStateAsColor(const uint8_t value)
  {
    std::array<float, 4> color;

    switch (value) {

      case MovementPhaseState::UNAVAILABLE:
        color = color_grey;
        break;

      case MovementPhaseState::DARK:
        color = color_grey;
        break;
      case MovementPhaseState::STOP_THEN_PROCEED:
        color = color_red;
        break;
      case MovementPhaseState::STOP_AND_REMAIN:
        color = color_red;
        break;
      case MovementPhaseState::PRE_MOVEMENT:
        color = color_orange;
        break;
      case MovementPhaseState::PERMISSIVE_MOVEMENT_ALLOWED:
        color = color_green;
        break;
      case MovementPhaseState::PROTECTED_MOVEMENT_ALLOWED:
        color = color_green;
        break;
      case MovementPhaseState::PERMISSIVE_CLEARANCE:
        color = color_orange;
        break;
      case MovementPhaseState::PROTECTED_CLEARANCE:
        color = color_orange;
        break;
      case MovementPhaseState::CAUTION_CONFLICTING_TRAFFIC:
        color = color_orange;
        break;
      default:
        color = color_grey;
        break;
  }

  return color;
}

/**
 * @brief Interprets the type of a TimeMark message
 * See etsi ASNI1 - IS TS 103 301 documentation for for the encoding of "TimeMark"
 * @param time The value inside the TimeMark message
 * @return Type as time_mark_value_interpretation 
 */
time_mark_value_interpretation interpretTimeMarkValueType(const uint16_t time) {
  time_mark_value_interpretation type;

  if (time == 36001) {
    // value is undefined or unknown
    type = time_mark_value_interpretation::undefined;
  } else if (time == 36000) {
    // used to indicate time >3600 seconds
    type = time_mark_value_interpretation::over_an_hour;
  } else if (time >= 35991 && time <= 35999) {
    // leap second
    type = time_mark_value_interpretation::leap_second;
  } else { // time >= 0 && time <= 36000
    type = time_mark_value_interpretation::normal;
  }

  return type;
}

/**
 * @brief Calculate the amount of seconds until the given time is reached
 * 
 * @param time Encoded time value in the future
 * @param seconds Elapsed seconds since the start of the last full hour (timestamp)
 * @param nanosec Elapsed nanoseconds since the start of the last full hour (timestamp)
 * @return Time in seconds refered to the given timestamp
 */
float interpretTimeMarkValueAsSeconds(const uint16_t time, const int32_t seconds, const uint32_t nanosec) {
  // calculate elapsed seconds since the start of the last full hour  
  float abs_time_hour = ((int)(seconds)) % 3600 + (float)nanosec * 1e-9;
  float rel_time_until_change = (float)time * 0.1f - abs_time_hour;
            
  return rel_time_until_change;
}

/**
 * @brief Converts a value from message type TimeMarkValue into a string representation
 * 
 * @param time Time in 0.1 seconds until the next change occours in the future, counting from the last started hour
 * @param seconds Elapsed seconds since the start of the last full hour (timestamp)
 * @param nanosec Elapsed nanoseconds since the start of the last full hour (timestamp)
 * @return Decoded String representation of the encoded time
 */
std::string parseTimeMarkValueToString(const uint16_t time, const int32_t seconds, const uint32_t nanosec)
{
  time_mark_value_interpretation time_type = interpretTimeMarkValueType(time);

  std::string text_content;

  switch (time_type) {
    case time_mark_value_interpretation::undefined:
      text_content = "undefined";
      break;
    case time_mark_value_interpretation::over_an_hour:
      text_content = ">36000s";
      break;
    case time_mark_value_interpretation::leap_second:
      text_content = "leap second";
      break;
    case time_mark_value_interpretation::normal:
      float rel_time_until_change = interpretTimeMarkValueAsSeconds(time, seconds, nanosec);

      // set displayed precision to 0.1
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << rel_time_until_change << "s";
      text_content = ss.str();
      break;
  }

  return text_content;
}


} // namespace etsi_its_spatem_ts_msgs
} // namespace access
