// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/cdd/cdd_v1-3-1_setters.h
 * @brief Setter functions for the ETSI ITS Common Data Dictionary (CDD) v1.3.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H

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
  throwIfOutOfRange(message_id, ItsPduHeader::MESSAGE_ID_MIN, ItsPduHeader::MESSAGE_ID_MAX, "MessageID");
  header.message_id = message_id;
  throwIfOutOfRange(protocol_version, ItsPduHeader::PROTOCOL_VERSION_MIN, ItsPduHeader::PROTOCOL_VERSION_MAX,
                    "ProtocolVersion");
  header.protocol_version = protocol_version;
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H