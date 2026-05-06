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
 * @file impl/cdd/cdd_v2-2-1_setters.h
 * @brief Setter functions for the ETSI ITS Common Data Dictionary (CDD) v2.2.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_2_1_SETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_2_1_SETTERS_H

#include <etsi_its_msgs_utils/impl/cdd/cdd_setters_common.h>
#include <etsi_its_msgs_utils/impl/checks.h>
#include <GeographicLib/UTMUPS.hpp>
#include <cstring>

/**
 * @brief Set the Its Pdu Header object
 *
 * @param header ItsPduHeader to be set
 * @param message_id ID of the message
 * @param station_id
 * @param protocol_version
 */
inline void setItsPduHeader(ItsPduHeader& header, const uint8_t message_id, const uint32_t station_id,
                            const uint8_t protocol_version = 0) {
  setStationId(header.station_id, station_id);
  throwIfOutOfRange(message_id, MessageId::MIN, MessageId::MAX, "MessageID");
  header.message_id.value = message_id;
  throwIfOutOfRange(protocol_version, OrdinalNumber1B::MIN, OrdinalNumber1B::MAX, "ProtocolVersion");
  header.protocol_version.value = protocol_version;
}

/**
 * @brief Set the Wgs84AngleValue object
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading object to set
 * @param value Heading value in degree as decimal number
 */
template <typename Wgs84AngleValue>
inline void setWGSHeadingValue(Wgs84AngleValue& heading, const double value) {
  int64_t deg = (int64_t)std::round(value * 1e1);
  throwIfOutOfRange(deg, Wgs84AngleValue::MIN, Wgs84AngleValue::MAX, "Wgs84AngleValue");
  heading.value = deg;
}

/**
 * @brief Set the Wgs84AngleConfidence object
 * 
 * @param confidence object to set
 * @param value standard deviation of heading in degree as decimal number
 */
template<typename Wgs84AngleConfidence>
inline void setWGSHeadingConfidence(Wgs84AngleConfidence& confidence, const double value) {
  auto heading_conf = std::round(value * 1e1 * etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR);
  if (heading_conf < Wgs84AngleConfidence::MIN && heading_conf > 0.0){
    heading_conf = Wgs84AngleConfidence::MIN;
  } else if (heading_conf >= Wgs84AngleConfidence::OUT_OF_RANGE || heading_conf <= 0.0) {
    heading_conf = Wgs84AngleConfidence::UNAVAILABLE;
  }
  confidence.value = static_cast<decltype(confidence.value)>(heading_conf);
}

/**
 * @brief Set the Wgs84Angle object
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 * Wgs84AngleConfidence is set to UNAVAILABLE
 *
 * @param heading object to set
 * @param value Heading value in degree as decimal number
 * @param confidence standard deviation of heading in degree as decimal number (default: infinity, mapping to Wgs84AngleConfidence::UNAVAILABLE) 
 */
template <typename Wgs84Angle, typename Wgs84AngleConfidence = decltype(Wgs84Angle::confidence)>
void setWGSHeadingCDD(Wgs84Angle& heading, const double value, double confidence = std::numeric_limits<double>::infinity()) {
  setWGSHeadingConfidence(heading.confidence, confidence);
  setWGSHeadingValue(heading.value, value);
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_2_1_SETTERS_H