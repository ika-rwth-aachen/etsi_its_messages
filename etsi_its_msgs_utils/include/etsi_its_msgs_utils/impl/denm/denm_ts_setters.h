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
 * @file impl/denm/denm_ts_setters.h
 * @brief Setter functions for the ETSI ITS DENM (TS)
 */

#pragma once

namespace etsi_its_denm_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-2-1_setters.h>

#include <etsi_its_msgs_utils/impl/denm/denm_setters_common.h>

/**
 * @brief Set the ItsPduHeader-object for a DENM
 *
 * @param denm DENM-Message to set the ItsPduHeader
 * @param station_id
 * @param protocol_version
 */
inline void setItsPduHeader(DENM& denm, const uint32_t station_id, const uint8_t protocol_version = 0) {
  setItsPduHeader(denm.header, MessageId::DENM, station_id, protocol_version);
}

/**
 * @brief Set the IsWGSHeadingPresent object for DENM
 * 
 * @param denm DENM to set IsWGSHeadingPresent
 * @param presence_of_heading IsWGSHeadingPresent-Value (true or false)
 */
inline void setIsWGSHeadingPresent(DENM& denm, bool presence_of_heading) {
  if (denm.denm.location_is_present) {
    denm.denm.location.event_position_heading_is_present = presence_of_heading;
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

/**
 * @brief Set the WGS Heading for a DENM
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 * Wgs84AngleConfidence is set to UNAVAILABLE
 *
 * @param denm DENM to set the WGS Heading
 * @param heading_val Heading value in degree as decimal number
 * @param confidence standard deviation of heading in degree as decimal number (default: infinity, mapping to Wgs84AngleConfidence::UNAVAILABLE)
 */
inline void setWGSHeading(DENM& denm, const double heading_val, const double confidence = std::numeric_limits<double>::infinity()) {
  if (denm.denm.location_is_present) {
    setWGSHeadingCDD(denm.denm.location.event_position_heading, heading_val, confidence);
    setIsWGSHeadingPresent(denm, true);
  } else {
    throw std::invalid_argument("LocationContainer is not present!");
  }
}

}  // namespace etsi_its_denm_ts_msgs::access
