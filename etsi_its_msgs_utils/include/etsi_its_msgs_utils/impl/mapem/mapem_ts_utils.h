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
 * @file impl/mapem/mapem_ts_utils.h
 * @brief Utility functions for the ETSI ITS MAPEM
 */

#include <ctime>
#include <GeographicLib/UTMUPS.hpp>

#pragma once

namespace etsi_its_mapem_ts_msgs {

namespace access {

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
   * @brief Get the unix nanoseconds from MapData object
   * 
   * @param map given MapData object
   * @param unix_nanoseconds_estimate unix timestamp to derive the current year from in nanoseconds
   * @return uint64_t unix timestamp according to the given MinuteOfTheYear in nanoseconds
   */
  inline uint64_t getUnixNanosecondsFromMapData(const MapData& map, const uint64_t unix_nanoseconds_estimate) {
    return getUnixNanosecondsFromMinuteOfTheYear(getMinuteOfTheYear(map), unix_nanoseconds_estimate);
  }

  /**
   * @brief Get the unix nanoseconds from MinuteOfTheYear object
   * 
   * @param mapem given MAPEM object
   * @param unix_nanoseconds_estimate unix timestamp to derive the current year from in nanoseconds
   * @return uint64_t unix timestamp according to the stored MinuteOfTheYear in nanoseconds
   */
  inline uint64_t getUnixNanoseconds(const MAPEM& mapem, const uint64_t unix_timestamp_estimate) {
    return getUnixNanosecondsFromMapData(mapem.map, unix_timestamp_estimate);
  }

  /**
   * @brief Get the UTM Position defined by the given Position3D
   *
   * The position is transformed into UTM by using GeographicLib::UTMUPS
   * The altitude value is directly used as z-Coordinate
   *
   * @param[in] reference_position Position3D to get the UTM Position from
   * @param[out] zone the UTM zone (zero means UPS)
   * @param[out] northp hemisphere (true means north, false means south)
   * @return gm::PointStamped geometry_msgs::PointStamped of the given position
   */
  inline gm::PointStamped getUTMPosition(const Position3D& reference_position, int& zone, bool& northp) {
    gm::PointStamped utm_point;
    double latitude = getLatitude(reference_position.lat);
    double longitude = getLongitude(reference_position.lon);
    if(reference_position.elevation_is_present) utm_point.point.z = getElevation(reference_position.elevation);
    try {
      GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.point.x, utm_point.point.y);
      std::string hemisphere;
      if(northp) hemisphere="N";
      else hemisphere="S";
      utm_point.header.frame_id="utm_"+std::to_string(zone)+hemisphere;
    } catch (GeographicLib::GeographicErr& e) {
      throw std::invalid_argument(e.what());
    }
    return utm_point;
  }

    /**
   * @brief Get the UTM Position defined by the given Position3D along with the grid-convergence angle
   *
   * The position is transformed into UTM by using GeographicLib::UTMUPS
   * The altitude value is directly used as z-Coordinate
   *
   * @param[in] reference_position Position3D to get the UTM Position from
   * @param[out] zone the UTM zone (zero means UPS)
   * @param[out] northp hemisphere (true means north, false means south)
   * @param[out] conv_angle grid-convergence angle in degree
   * @return gm::PointStamped geometry_msgs::PointStamped of the given position
   */
  inline gm::PointStamped getUTMPositionWithConvergenceAngle(const Position3D& reference_position, int& zone, bool& northp, double& conv_angle) {
    gm::PointStamped utm_point;
    double latitude = getLatitude(reference_position.lat);
    double longitude = getLongitude(reference_position.lon);
    if(reference_position.elevation_is_present) utm_point.point.z = getElevation(reference_position.elevation);
    try {
      double scale;
      GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.point.x, utm_point.point.y, conv_angle, scale);
      std::string hemisphere;
      if(northp) hemisphere="N";
      else hemisphere="S";
      utm_point.header.frame_id="utm_"+std::to_string(zone)+hemisphere;
    } catch (GeographicLib::GeographicErr& e) {
      throw std::invalid_argument(e.what());
    }
    return utm_point;
  }

  /**
   * @brief Get the UTM Position of ref_point defined by the Position3D along with the grid-convergence angle in an IntersectionGeometry object
   *
   * The position is transformed into UTM by using GeographicLib::UTMUPS
   * The altitude value is directly used as z-Coordinate
   *
   * @param[in] intsctn IntersectionGeometry to get the UTM Position from
   * @param[out] zone the UTM zone (zero means UPS)
   * @param[out] northp hemisphere (true means north, false means south)
   * @return gm::PointStamped geometry_msgs::PointStamped of the given position
   */
  inline gm::PointStamped getRefPointUTMPosition(const IntersectionGeometry& intsctn, int& zone, bool& northp) {
    return getUTMPosition(intsctn.ref_point, zone, northp);
  }

    /**
   * @brief Get the UTM Position of ref_point defined by the Position3D in an IntersectionGeometry object
   *
   * The position is transformed into UTM by using GeographicLib::UTMUPS
   * The altitude value is directly used as z-Coordinate
   *
   * @param[in] intsctn IntersectionGeometry to get the UTM Position from
   * @param[out] zone the UTM zone (zero means UPS)
   * @param[out] northp hemisphere (true means north, false means south)
   * @param[out] conv_angle grid-convergence angle in degree
   * @return gm::PointStamped geometry_msgs::PointStamped of the given position
   */
  inline gm::PointStamped getRefPointUTMPositionWithConvergenceAngle(const IntersectionGeometry& intsctn, int& zone, bool& northp, double& conv_angle) {
    return getUTMPositionWithConvergenceAngle(intsctn.ref_point, zone, northp, conv_angle);
  }

} // namespace etsi_its_mapem_ts_msgs
} // namespace access
