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
 * @file impl/cpm/cpm_ts_setters.h
 * @brief Setter functions for the ETSI ITS CPM (TS)
 */

#pragma once

#include <etsi_its_msgs_utils/impl/constants.h>

namespace etsi_its_cpm_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-1-1_setters.h>

  /**
   * @brief Set the ItsPduHeader-object for a CPM
   *
   * @param cpm CPM-Message to set the ItsPduHeader
   * @param station_id
   * @param protocol_version
   */
  inline void setItsPduHeader(CollectivePerceptionMessage& cpm, const uint32_t station_id, const uint8_t protocol_version = 0){
    setItsPduHeader(cpm.header, MessageId::CPM, station_id, protocol_version);
  }

    /**
   * @brief Set the ReferencePositionWithConfidence for a CPM TS
   *
   * This function sets the latitude, longitude, and altitude of the CPMs reference position.
   * If the altitude is not provided, it is set to AltitudeValue::UNAVAILABLE.
   *
   * @param cpm CPM to set the ReferencePosition
   * @param latitude The latitude value position in degree as decimal number.
   * @param longitude The longitude value in degree as decimal number.
   * @param altitude The altitude value (above the reference ellipsoid surface) in meter as decimal number (optional).
   */
  inline void setReferencePosition(CollectivePerceptionMessage& cpm, const double latitude, const double longitude, const double altitude = AltitudeValue::UNAVAILABLE) {
    setReferencePosition(cpm.payload.management_container.reference_position, latitude, longitude, altitude);
  }

  /**
   * @brief Set the ReferencePosition of a CPM from a given UTM-Position
   *
   * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
   * The z-Coordinate is directly used as altitude value
   * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
   *
   * @param[out] cpm CPM for which to set the ReferencePosition
   * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
   * @param[in] zone the UTM zone (zero means UPS) of the given position
   * @param[in] northp hemisphere (true means north, false means south)
   */
  inline void setFromUTMPosition(CollectivePerceptionMessage& cpm, const gm::PointStamped& utm_position, const int& zone, const bool& northp) {
    setFromUTMPosition(cpm.payload.management_container.reference_position, utm_position, zone, northp);
  }

} // namespace etsi_its_cpm_ts_msgs::access
