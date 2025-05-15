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
 * @file impl/mcm/mcm_getters.h
 * @brief Getter functions for the UULM MCM (TS)
 */

#pragma once

namespace etsi_its_mcm_uulm_msgs::access {
// #include <etsi_its_msgs_utils/impl/cdd/cdd_getters_common.h>

/**
 * @brief Get the Latitude value
 *
 * @param latitude to get the Latitude value from
 * @return Latitude value in degree as decimal number
 */
 inline double getLatitude(const Latitude& latitude) { return ((double)latitude.value) * 1e-7; }

 /**
  * @brief Get the Longitude value
  *
  * @param longitude to get the Longitude value from
  * @return Longitude value in degree as decimal number
  */
 inline double getLongitude(const Longitude& longitude) { return ((double)longitude.value) * 1e-7; }
 
 /**
  * @brief Get the Altitude value
  *
  * @param altitude to get the Altitude value from
  * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
  */
 inline double getAltitude(const Altitude& altitude) { return ((double)altitude.altitude_value.value) * 1e-2; }

/**
 * @brief Get the UTM Position defined by the given ReferencePosition
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] reference_position ReferencePosition or ReferencePositionWithConfidence to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
 template <typename T>
 inline gm::PointStamped getUTMPosition(const T& reference_position, int& zone, bool& northp) {
   gm::PointStamped utm_point;
   double latitude = getLatitude(reference_position.latitude);
   double longitude = getLongitude(reference_position.longitude);
   utm_point.point.z = getAltitude(reference_position.altitude);
   try {
     GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.point.x, utm_point.point.y);
     std::string hemisphere;
     if (northp) {
       hemisphere = "N";
     } else {
       hemisphere = "S";
     }
     utm_point.header.frame_id = "utm_" + std::to_string(zone) + hemisphere;
   } catch (GeographicLib::GeographicErr& e) {
     throw std::invalid_argument(e.what());
   }
   return utm_point;
 }

/**
 * @brief Get the UTM Position defined within the BasicContainer of the MCM
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] mcm MCM to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
 inline gm::PointStamped getUTMPosition(const MCM& mcm, int& zone, bool& northp) {
    return getUTMPosition(mcm.mcm.mcm_parameters.basic_container.reference_position, zone, northp);
 }

/**
 * @brief Get the UTM Position defined within the BasicContainer of the CAM
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] mcm MCM to get the UTM Position from
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
 inline gm::PointStamped getUTMPosition(const MCM& mcm) {
    int zone;
    bool northp;
    return getUTMPosition(mcm.mcm.mcm_parameters.basic_container.reference_position, zone, northp);
  }  


}  // namespace etsi_its_mcm_uulm_msgs::access