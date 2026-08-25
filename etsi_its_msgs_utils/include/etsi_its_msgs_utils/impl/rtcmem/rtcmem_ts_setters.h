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
 * @file impl/rtcmem/rtcmem_ts_setters.h
 * @brief Setter functions for the ETSI ITS RTCMEM
 */

#pragma once

#include <cmath>
#include <vector>

namespace etsi_its_rtcmem_ts_msgs {

namespace access {

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_setters.h>
#include <etsi_its_msgs_utils/impl/checks.h>

  /**
   * @brief Set the protocol version of an ItsPduHeader object
   *
   * @param header ItsPduHeader object to set
   * @param value protocol version value
   */
  inline void setProtocolVersion(ItsPduHeader& header, const uint8_t value) {
    throwIfOutOfRange(value, ItsPduHeader::PROTOCOL_VERSION_MIN, ItsPduHeader::PROTOCOL_VERSION_MAX,
                      "ItsPduHeader.protocol_version");
    header.protocol_version = value;
  }

  /**
   * @brief Set the message ID of an ItsPduHeader object
   *
   * @param header ItsPduHeader object to set
   * @param value message ID value
   */
  inline void setMessageID(ItsPduHeader& header, const uint8_t value) {
    throwIfOutOfRange(value, ItsPduHeader::MESSAGE_ID_MIN, ItsPduHeader::MESSAGE_ID_MAX,
                      "ItsPduHeader.message_id");
    header.message_id = value;
  }

  /**
   * @brief Set the message ID of an ItsPduHeader object to RTCMEM
   *
   * @param header ItsPduHeader object to set
   */
  inline void setMessageIDToRTCMEM(ItsPduHeader& header) {
    header.message_id = ItsPduHeader::MESSAGE_ID_RTCMEM;
  }

  /**
   * @brief Set the StationID object
   *
   * @param station_id StationID object to set
   * @param value station ID value
   */
  inline void setStationID(StationID& station_id, const uint32_t value) {
    throwIfOutOfRange(value, StationID::MIN, StationID::MAX, "StationID");
    station_id.value = value;
  }

  /**
   * @brief Set the station ID of an ItsPduHeader object
   *
   * @param header ItsPduHeader object to set
   * @param value station ID value
   */
  inline void setStationID(ItsPduHeader& header, const uint32_t value) {
    setStationID(header.station_id, value);
  }

  /**
   * @brief Set the ItsPduHeader of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param header ItsPduHeader object
   */
  inline void setHeader(RTCMEM& rtcmem, const ItsPduHeader& header) {
    rtcmem.header = header;
  }

  /**
   * @brief Set the protocol version of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param value protocol version value
   */
  inline void setProtocolVersion(RTCMEM& rtcmem, const uint8_t value) {
    setProtocolVersion(rtcmem.header, value);
  }

  /**
   * @brief Set the message ID of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param value message ID value
   */
  inline void setMessageID(RTCMEM& rtcmem, const uint8_t value) {
    setMessageID(rtcmem.header, value);
  }

  /**
   * @brief Set the message ID of a RTCMEM object to RTCMEM
   *
   * @param rtcmem RTCMEM object to set
   */
  inline void setMessageIDToRTCMEM(RTCMEM& rtcmem) {
    setMessageIDToRTCMEM(rtcmem.header);
  }

  /**
   * @brief Set the station ID of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param value station ID value
   */
  inline void setStationID(RTCMEM& rtcmem, const uint32_t value) {
    setStationID(rtcmem.header, value);
  }

  /**
   * @brief Set the RTCMcorrections container of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param rtcm RTCMcorrections object
   */
  inline void setRTCMcorrections(RTCMEM& rtcmem, const RTCMcorrections& rtcm) {
    rtcmem.rtcmc = rtcm;
  }

  /**
   * @brief Set the MsgCount object
   *
   * @param msg_cnt MsgCount object to set
   * @param value MsgCount value
   */
  inline void setMsgCount(MsgCount& msg_cnt, const uint8_t value) {
    throwIfOutOfRange(value, MsgCount::MIN, MsgCount::MAX, "MsgCount");
    msg_cnt.value = value;
  }

  /**
   * @brief Set the MsgCount of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to set
   * @param value MsgCount value
   */
  inline void setMsgCount(RTCMcorrections& rtcm, const uint8_t value) {
    setMsgCount(rtcm.msg_cnt, value);
  }

  /**
   * @brief Set the MsgCount of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param value MsgCount value
   */
  inline void setMsgCount(RTCMEM& rtcmem, const uint8_t value) {
    setMsgCount(rtcmem.rtcmc, value);
  }

  /**
   * @brief Set the RTCMRevision value
   *
   * @param rev RTCMRevision object to set
   * @param value RTCM revision value
   */
  inline void setRTCMRevision(RTCMRevision& rev, const uint8_t value) {
    throwIfOutOfRange(value, RTCMRevision::UNKNOWN, RTCMRevision::RESERVED, "RTCMRevision");
    rev.value = value;
  }

  /**
   * @brief Set the RTCMRevision of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to set
   * @param value RTCM revision value
   */
  inline void setRTCMRevision(RTCMcorrections& rtcm, const uint8_t value) {
    setRTCMRevision(rtcm.rev, value);
  }

  /**
   * @brief Set the RTCMRevision of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param value RTCM revision value
   */
  inline void setRTCMRevision(RTCMEM& rtcmem, const uint8_t value) {
    setRTCMRevision(rtcmem.rtcmc, value);
  }

  /**
   * @brief Set the MinuteOfTheYear object
   *
   * @param moy MinuteOfTheYear object to set
   * @param value MinuteOfTheYear value
   */
  inline void setMinuteOfTheYear(MinuteOfTheYear& moy, const uint32_t value) {
    throwIfOutOfRange(value, MinuteOfTheYear::MIN, MinuteOfTheYear::MAX, "MinuteOfTheYear");
    moy.value = value;
  }

  /**
   * @brief Set the MinuteOfTheYear of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to set
   * @param value MinuteOfTheYear value
   */
  inline void setMinuteOfTheYear(RTCMcorrections& rtcm, const uint32_t value) {
    setMinuteOfTheYear(rtcm.time_stamp, value);
    rtcm.time_stamp_is_present = true;
  }

  /**
   * @brief Set the MinuteOfTheYear of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param value MinuteOfTheYear value
   */
  inline void setMinuteOfTheYear(RTCMEM& rtcmem, const uint32_t value) {
    setMinuteOfTheYear(rtcmem.rtcmc, value);
  }

  /**
   * @brief Clear the optional MinuteOfTheYear of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to update
   */
  inline void clearMinuteOfTheYear(RTCMcorrections& rtcm) {
    rtcm.time_stamp_is_present = false;
  }

  /**
   * @brief Clear the optional MinuteOfTheYear of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to update
   */
  inline void clearMinuteOfTheYear(RTCMEM& rtcmem) {
    clearMinuteOfTheYear(rtcmem.rtcmc);
  }

  /**
   * @brief Set the anchor point of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to set
   * @param anchor anchor point object
   */
  inline void setAnchorPoint(RTCMcorrections& rtcm, const FullPositionVector& anchor) {
    rtcm.anchor_point = anchor;
    rtcm.anchor_point_is_present = true;
  }

  /**
   * @brief Set the anchor point of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param anchor anchor point object
   */
  inline void setAnchorPoint(RTCMEM& rtcmem, const FullPositionVector& anchor) {
    setAnchorPoint(rtcmem.rtcmc, anchor);
  }

  /**
   * @brief Clear the optional anchor point of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to update
   */
  inline void clearAnchorPoint(RTCMcorrections& rtcm) {
    rtcm.anchor_point_is_present = false;
  }

  /**
   * @brief Clear the optional anchor point of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to update
   */
  inline void clearAnchorPoint(RTCMEM& rtcmem) {
    clearAnchorPoint(rtcmem.rtcmc);
  }

  /**
   * @brief Set the RTCMheader of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to set
   * @param header RTCMheader object
   */
  inline void setRTCMHeader(RTCMcorrections& rtcm, const RTCMheader& header) {
    rtcm.rtcm_header = header;
    rtcm.rtcm_header_is_present = true;
  }

  /**
   * @brief Set the RTCMheader of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param header RTCMheader object
   */
  inline void setRTCMHeader(RTCMEM& rtcmem, const RTCMheader& header) {
    setRTCMHeader(rtcmem.rtcmc, header);
  }

  /**
   * @brief Clear the optional RTCMheader of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to update
   */
  inline void clearRTCMHeader(RTCMcorrections& rtcm) {
    rtcm.rtcm_header_is_present = false;
  }

  /**
   * @brief Clear the optional RTCMheader of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to update
   */
  inline void clearRTCMHeader(RTCMEM& rtcmem) {
    clearRTCMHeader(rtcmem.rtcmc);
  }

  /**
   * @brief Set the raw bytes of a RTCMmessage object
   *
   * @param msg RTCMmessage object to set
   * @param data RTCM payload bytes
   */
  inline void setRTCMmessageValue(RTCMmessage& msg, const std::vector<uint8_t>& data) {
    throwIfOutOfRange(data.size(),
                      static_cast<std::size_t>(RTCMmessage::MIN_SIZE),
                      static_cast<std::size_t>(RTCMmessage::MAX_SIZE),
                      "RTCMmessage length");
    msg.value = data;
  }

  /**
   * @brief Set the raw bytes of a RTCMmessage object
   *
   * @param msg RTCMmessage object to set
   * @param data RTCM payload bytes
   */
  inline void setRTCMmessageData(RTCMmessage& msg, const std::vector<uint8_t>& data) {
    setRTCMmessageValue(msg, data);
  }

  /**
   * @brief Set the RTCMmessageList object
   *
   * @param msgs RTCMmessageList object to set
   * @param values RTCM messages
   */
  inline void setRTCMmessageList(RTCMmessageList& msgs, const std::vector<RTCMmessage>& values) {
    throwIfOutOfRange(values.size(),
                      static_cast<std::size_t>(RTCMmessageList::MIN_SIZE),
                      static_cast<std::size_t>(RTCMmessageList::MAX_SIZE),
                      "RTCMmessageList size");
    msgs.array = values;
  }

  /**
   * @brief Set the RTCMmessageList of a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to set
   * @param values RTCM messages
   */
  inline void setRTCMmessageList(RTCMcorrections& rtcm, const std::vector<RTCMmessage>& values) {
    setRTCMmessageList(rtcm.msgs, values);
  }

  /**
   * @brief Set the RTCMmessageList of a RTCMEM object
   *
   * @param rtcmem RTCMEM object to set
   * @param values RTCM messages
   */
  inline void setRTCMmessageList(RTCMEM& rtcmem, const std::vector<RTCMmessage>& values) {
    setRTCMmessageList(rtcmem.rtcmc, values);
  }

  /**
   * @brief Add a new default-constructed RTCMmessage to a RTCMmessageList object
   *
   * @param msgs RTCMmessageList object to extend
   * @return RTCMmessage& the newly added RTCMmessage
   */
  inline RTCMmessage& addRTCMmessage(RTCMmessageList& msgs) {
    throwIfOutOfRange(msgs.array.size() + 1,
                      static_cast<std::size_t>(RTCMmessageList::MIN_SIZE),
                      static_cast<std::size_t>(RTCMmessageList::MAX_SIZE),
                      "RTCMmessageList size");
    msgs.array.emplace_back();
    return msgs.array.back();
  }

  /**
   * @brief Add a new default-constructed RTCMmessage to a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to extend
   * @return RTCMmessage& the newly added RTCMmessage
   */
  inline RTCMmessage& addRTCMmessage(RTCMcorrections& rtcm) {
    return addRTCMmessage(rtcm.msgs);
  }

  /**
   * @brief Add a new default-constructed RTCMmessage to a RTCMEM object
   *
   * @param rtcmem RTCMEM object to extend
   * @return RTCMmessage& the newly added RTCMmessage
   */
  inline RTCMmessage& addRTCMmessage(RTCMEM& rtcmem) {
    return addRTCMmessage(rtcmem.rtcmc);
  }

  /**
   * @brief Add a new RTCMmessage with payload to a RTCMmessageList object
   *
   * @param msgs RTCMmessageList object to extend
   * @param data RTCM payload bytes
   * @return RTCMmessage& the newly added RTCMmessage
   */
  inline RTCMmessage& addRTCMmessage(RTCMmessageList& msgs, const std::vector<uint8_t>& data) {
    RTCMmessage& msg = addRTCMmessage(msgs);
    setRTCMmessageValue(msg, data);
    return msg;
  }

  /**
   * @brief Add a new RTCMmessage with payload to a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to extend
   * @param data RTCM payload bytes
   * @return RTCMmessage& the newly added RTCMmessage
   */
  inline RTCMmessage& addRTCMmessage(RTCMcorrections& rtcm, const std::vector<uint8_t>& data) {
    return addRTCMmessage(rtcm.msgs, data);
  }

  /**
   * @brief Add a new RTCMmessage with payload to a RTCMEM object
   *
   * @param rtcmem RTCMEM object to extend
   * @param data RTCM payload bytes
   * @return RTCMmessage& the newly added RTCMmessage
   */
  inline RTCMmessage& addRTCMmessage(RTCMEM& rtcmem, const std::vector<uint8_t>& data) {
    return addRTCMmessage(rtcmem.rtcmc, data);
  }

  /**
   * @brief Clear all RTCM messages from a RTCMmessageList object
   *
   * @param msgs RTCMmessageList object to clear
   */
  inline void clearRTCMmessages(RTCMmessageList& msgs) {
    msgs.array.clear();
  }

  /**
   * @brief Clear all RTCM messages from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to clear
   */
  inline void clearRTCMmessages(RTCMcorrections& rtcm) {
    clearRTCMmessages(rtcm.msgs);
  }

  /**
   * @brief Clear all RTCM messages from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to clear
   */
  inline void clearRTCMmessages(RTCMEM& rtcmem) {
    clearRTCMmessages(rtcmem.rtcmc);
  }

  /**
   * @brief Set the GNSSstatus object of a RTCMheader object
   *
   * @param header RTCMheader object to set
   * @param status GNSSstatus object
   */
  inline void setGNSSstatus(RTCMheader& header, const GNSSstatus& status) {
    header.status = status;
  }

  /**
   * @brief Set the GNSSstatus of a RTCMheader object from a bool vector
   *
   * @param header RTCMheader object to set
   * @param status_bits GNSSstatus bits
   */
  inline void setGNSSstatus(RTCMheader& header, const std::vector<bool>& status_bits) {
    setBitString(header.status, status_bits);
  }

  /**
   * @brief Set the AntennaOffsetSet object of a RTCMheader object
   *
   * @param header RTCMheader object to set
   * @param offset_set AntennaOffsetSet object
   */
  inline void setAntennaOffsetSet(RTCMheader& header, const AntennaOffsetSet& offset_set) {
    header.offset_set = offset_set;
  }

  /**
   * @brief Ensure storage for a GNSSstatus object
   *
   * @param status GNSSstatus object to prepare
   */
  inline void ensureGNSSstatusStorage(GNSSstatus& status) {
    if (status.value.size() < 1) {
      status.value.resize(1, 0u);
      status.bits_unused = 0;
    }
  }

  /**
   * @brief Set a single GNSSstatus bit
   *
   * @param status GNSSstatus object to update
   * @param bit_index bit index to write
   * @param value bit value
   */
  inline void setGNSSstatusBit(GNSSstatus& status, const uint8_t bit_index, const bool value) {
    throwIfOutOfRange(bit_index, static_cast<uint8_t>(0), static_cast<uint8_t>(GNSSstatus::SIZE_BITS - 1), "GNSSstatus bit index");

    ensureGNSSstatusStorage(status);
    const uint8_t mask = static_cast<uint8_t>(1u << bit_index);

    if (value) {
      status.value[0] = static_cast<uint8_t>(status.value[0] | mask);
    } else {
      status.value[0] = static_cast<uint8_t>(status.value[0] & ~mask);
    }
  }

  /**
   * @brief Set the unavailable flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setUnavailableFlag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_UNAVAILABLE, value);
  }

  /**
   * @brief Set the isHealthy flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setIsHealthyFlag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_IS_HEALTHY, value);
  }

  /**
   * @brief Set the isMonitored flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setIsMonitoredFlag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_IS_MONITORED, value);
  }

  /**
   * @brief Set the baseStationType flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setBaseStationTypeFlag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_BASE_STATION_TYPE, value);
  }

  /**
   * @brief Set the aPDOPofUnder5 flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setAPDOPofUnder5Flag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_A_PDO_POF_UNDER5, value);
  }

  /**
   * @brief Set the inViewOfUnder5 flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setInViewOfUnder5Flag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_IN_VIEW_OF_UNDER5, value);
  }

  /**
   * @brief Set the localCorrectionsPresent flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setLocalCorrectionsPresentFlag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_LOCAL_CORRECTIONS_PRESENT, value);
  }

  /**
   * @brief Set the networkCorrectionsPresent flag of a GNSSstatus object
   *
   * @param status GNSSstatus object to update
   * @param value flag value
   */
  inline void setNetworkCorrectionsPresentFlag(GNSSstatus& status, const bool value) {
    setGNSSstatusBit(status, GNSSstatus::BIT_INDEX_NETWORK_CORRECTIONS_PRESENT, value);
  }

  /**
   * @brief Set the x antenna offset object
   *
   * @param field OffsetB12 object to set
   * @param value offset value
   */
  inline void setAntennaOffsetX(OffsetB12& field, const int16_t value) {
    throwIfOutOfRange(value, OffsetB12::MIN, OffsetB12::MAX, "OffsetB12");
    field.value = value;
  }

  /**
   * @brief Set the y antenna offset object
   *
   * @param field OffsetB09 object to set
   * @param value offset value
   */
  inline void setAntennaOffsetY(OffsetB09& field, const int16_t value) {
    throwIfOutOfRange(value, OffsetB09::MIN, OffsetB09::MAX, "OffsetB09");
    field.value = value;
  }

  /**
   * @brief Set the z antenna offset object
   *
   * @param field OffsetB10 object to set
   * @param value offset value
   */
  inline void setAntennaOffsetZ(OffsetB10& field, const int16_t value) {
    throwIfOutOfRange(value, OffsetB10::MIN, OffsetB10::MAX, "OffsetB10");
    field.value = value;
  }

  /**
   * @brief Set the x antenna offset of an AntennaOffsetSet object
   *
   * @param offset AntennaOffsetSet object to set
   * @param value offset value
   */
  inline void setAntennaOffsetX(AntennaOffsetSet& offset, const int16_t value) {
    setAntennaOffsetX(offset.ant_offset_x, value);
  }

  /**
   * @brief Set the y antenna offset of an AntennaOffsetSet object
   *
   * @param offset AntennaOffsetSet object to set
   * @param value offset value
   */
  inline void setAntennaOffsetY(AntennaOffsetSet& offset, const int16_t value) {
    setAntennaOffsetY(offset.ant_offset_y, value);
  }

  /**
   * @brief Set the z antenna offset of an AntennaOffsetSet object
   *
   * @param offset AntennaOffsetSet object to set
   * @param value offset value
   */
  inline void setAntennaOffsetZ(AntennaOffsetSet& offset, const int16_t value) {
    setAntennaOffsetZ(offset.ant_offset_z, value);
  }

  /**
   * @brief Set the raw longitude value of a Longitude object
   *
   * @param longitude Longitude object to set
   * @param value raw longitude value
   */
  inline void setLongitude(Longitude& longitude, const int32_t value) {
    throwIfOutOfRange(value, Longitude::MIN, Longitude::MAX, "Longitude");
    longitude.value = value;
  }

  /**
   * @brief Set the longitude of a Longitude object in degree
   *
   * @param longitude Longitude object to set
   * @param value longitude in degree
   */
  inline void setLongitude(Longitude& longitude, const double value) {
    int64_t longitude_value = static_cast<int64_t>(std::round(value * 1e7));
    throwIfOutOfRange(longitude_value, Longitude::MIN, Longitude::MAX, "Longitude");
    longitude.value = longitude_value;
  }

  /**
   * @brief Set the raw latitude value of a Latitude object
   *
   * @param latitude Latitude object to set
   * @param value raw latitude value
   */
  inline void setLatitude(Latitude& latitude, const int32_t value) {
    throwIfOutOfRange(value, Latitude::MIN, Latitude::MAX, "Latitude");
    latitude.value = value;
  }

  /**
   * @brief Set the latitude of a Latitude object in degree
   *
   * @param latitude Latitude object to set
   * @param value latitude in degree
   */
  inline void setLatitude(Latitude& latitude, const double value) {
    int64_t latitude_value = static_cast<int64_t>(std::round(value * 1e7));
    throwIfOutOfRange(latitude_value, Latitude::MIN, Latitude::MAX, "Latitude");
    latitude.value = latitude_value;
  }

  /**
   * @brief Set the raw elevation value of an Elevation object
   *
   * @param elevation Elevation object to set
   * @param value raw elevation value
   */
  inline void setElevation(Elevation& elevation, const int32_t value) {
    throwIfOutOfRange(value, Elevation::MIN, Elevation::MAX, "Elevation");
    elevation.value = value;
  }

  /**
   * @brief Set the elevation of an Elevation object in meter
   *
   * @param elevation Elevation object to set
   * @param value elevation in meter
   */
  inline void setElevation(Elevation& elevation, const double value) {
    int64_t elevation_value = static_cast<int64_t>(std::round(value * 1e1));
    throwIfOutOfRange(elevation_value, Elevation::MIN, Elevation::MAX, "Elevation");
    elevation.value = elevation_value;
  }

  /**
   * @brief Set the heading of a HeadingDSRC object
   *
   * @param heading HeadingDSRC object to set
   * @param value heading value
   */
  inline void setHeading(HeadingDSRC& heading, const uint16_t value) {
    throwIfOutOfRange(value, HeadingDSRC::MIN, HeadingDSRC::MAX, "HeadingDSRC");
    heading.value = value;
  }

  /**
   * @brief Set the utcTime of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param utc_time DDateTime object
   */
  inline void setUTCTime(FullPositionVector& pos, const DDateTime& utc_time) {
    pos.utc_time = utc_time;
    pos.utc_time_is_present = true;
  }

  /**
   * @brief Clear the optional utcTime of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearUTCTime(FullPositionVector& pos) {
    pos.utc_time_is_present = false;
  }

  /**
   * @brief Set the longitude of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param value raw longitude value
   */
  inline void setLongitude(FullPositionVector& pos, const int32_t value) {
    setLongitude(pos.lon, value);
  }

  /**
   * @brief Set the longitude of a FullPositionVector object in degree
   *
   * @param pos FullPositionVector object to set
   * @param value longitude in degree
   */
  inline void setLongitude(FullPositionVector& pos, const double value) {
    setLongitude(pos.lon, value);
  }

  /**
   * @brief Set the latitude of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param value raw latitude value
   */
  inline void setLatitude(FullPositionVector& pos, const int32_t value) {
    setLatitude(pos.lat, value);
  }

  /**
   * @brief Set the latitude of a FullPositionVector object in degree
   *
   * @param pos FullPositionVector object to set
   * @param value latitude in degree
   */
  inline void setLatitude(FullPositionVector& pos, const double value) {
    setLatitude(pos.lat, value);
  }

  /**
   * @brief Set the Elevation object of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param elevation Elevation object
   */
  inline void setElevation(FullPositionVector& pos, const Elevation& elevation) {
    pos.elevation = elevation;
    pos.elevation_is_present = true;
  }

  /**
   * @brief Set the elevation of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param value raw elevation value
   */
  inline void setElevation(FullPositionVector& pos, const int32_t value) {
    setElevation(pos.elevation, value);
    pos.elevation_is_present = true;
  }

  /**
   * @brief Set the elevation of a FullPositionVector object in meter
   *
   * @param pos FullPositionVector object to set
   * @param value elevation in meter
   */
  inline void setElevation(FullPositionVector& pos, const double value) {
    setElevation(pos.elevation, value);
    pos.elevation_is_present = true;
  }

  /**
   * @brief Clear the optional elevation of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearElevation(FullPositionVector& pos) {
    pos.elevation_is_present = false;
  }

  /**
   * @brief Set the heading of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param heading HeadingDSRC object
   */
  inline void setHeading(FullPositionVector& pos, const HeadingDSRC& heading) {
    pos.heading = heading;
    pos.heading_is_present = true;
  }

  /**
   * @brief Set the heading of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param value heading value
   */
  inline void setHeading(FullPositionVector& pos, const uint16_t value) {
    setHeading(pos.heading, value);
    pos.heading_is_present = true;
  }

  /**
   * @brief Clear the optional heading of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearHeading(FullPositionVector& pos) {
    pos.heading_is_present = false;
  }

  /**
   * @brief Set the speed of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param speed TransmissionAndSpeed object
   */
  inline void setSpeed(FullPositionVector& pos, const TransmissionAndSpeed& speed) {
    pos.speed = speed;
    pos.speed_is_present = true;
  }

  /**
   * @brief Clear the optional speed of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearSpeed(FullPositionVector& pos) {
    pos.speed_is_present = false;
  }

  /**
   * @brief Set the positional accuracy of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param accuracy PositionalAccuracy object
   */
  inline void setPositionalAccuracy(FullPositionVector& pos, const PositionalAccuracy& accuracy) {
    pos.pos_accuracy = accuracy;
    pos.pos_accuracy_is_present = true;
  }

  /**
   * @brief Clear the optional positional accuracy of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearPositionalAccuracy(FullPositionVector& pos) {
    pos.pos_accuracy_is_present = false;
  }

  /**
   * @brief Set the time confidence value
   *
   * @param time_confidence TimeConfidence object to set
   * @param value time confidence value
   */
  inline void setTimeConfidence(TimeConfidence& time_confidence, const uint8_t value) {
    throwIfOutOfRange(value, TimeConfidence::UNAVAILABLE, TimeConfidence::TIME_000_000_000_01,
                      "TimeConfidence");
    time_confidence.value = value;
  }

  /**
   * @brief Set the time confidence of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param time_confidence TimeConfidence object
   */
  inline void setTimeConfidence(FullPositionVector& pos, const TimeConfidence& time_confidence) {
    pos.time_confidence = time_confidence;
    pos.time_confidence_is_present = true;
  }

  /**
   * @brief Set the time confidence of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param value time confidence value
   */
  inline void setTimeConfidence(FullPositionVector& pos, const uint8_t value) {
    setTimeConfidence(pos.time_confidence, value);
    pos.time_confidence_is_present = true;
  }

  /**
   * @brief Clear the optional time confidence of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearTimeConfidence(FullPositionVector& pos) {
    pos.time_confidence_is_present = false;
  }

  /**
   * @brief Set the position confidence of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param confidence PositionConfidenceSet object
   */
  inline void setPositionConfidence(FullPositionVector& pos, const PositionConfidenceSet& confidence) {
    pos.pos_confidence = confidence;
    pos.pos_confidence_is_present = true;
  }

  /**
   * @brief Clear the optional position confidence of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearPositionConfidence(FullPositionVector& pos) {
    pos.pos_confidence_is_present = false;
  }

  /**
   * @brief Set the speed confidence of a FullPositionVector object
   *
   * @param pos FullPositionVector object to set
   * @param confidence SpeedandHeadingandThrottleConfidence object
   */
  inline void setSpeedConfidence(FullPositionVector& pos,
                                 const SpeedandHeadingandThrottleConfidence& confidence) {
    pos.speed_confidence = confidence;
    pos.speed_confidence_is_present = true;
  }

  /**
   * @brief Clear the optional speed confidence of a FullPositionVector object
   *
   * @param pos FullPositionVector object to update
   */
  inline void clearSpeedConfidence(FullPositionVector& pos) {
    pos.speed_confidence_is_present = false;
  }

  /**
   * @brief Set the transmission state value
   *
   * @param transmission_state TransmissionState object to set
   * @param value transmission state value
   */
  inline void setTransmissionState(TransmissionState& transmission_state, const uint8_t value) {
    throwIfOutOfRange(value, TransmissionState::NEUTRAL, TransmissionState::UNAVAILABLE, "TransmissionState");
    transmission_state.value = value;
  }

  /**
   * @brief Set the transmission state of a TransmissionAndSpeed object
   *
   * @param speed TransmissionAndSpeed object to set
   * @param value transmission state value
   */
  inline void setTransmissionState(TransmissionAndSpeed& speed, const uint8_t value) {
    setTransmissionState(speed.transmisson, value);
  }

  /**
   * @brief Set the velocity value
   *
   * @param velocity Velocity object to set
   * @param value velocity value
   */
  inline void setVelocity(Velocity& velocity, const uint16_t value) {
    throwIfOutOfRange(value, Velocity::MIN, Velocity::MAX, "Velocity");
    velocity.value = value;
  }

  /**
   * @brief Set the velocity of a TransmissionAndSpeed object
   *
   * @param speed TransmissionAndSpeed object to set
   * @param value velocity value
   */
  inline void setVelocity(TransmissionAndSpeed& speed, const uint16_t value) {
    setVelocity(speed.speed, value);
  }

  /**
   * @brief Set the transmission state and velocity of a TransmissionAndSpeed object
   *
   * @param speed TransmissionAndSpeed object to set
   * @param transmission_state transmission state value
   * @param velocity velocity value
   */
  inline void setTransmissionAndSpeed(TransmissionAndSpeed& speed, const uint8_t transmission_state,
                                      const uint16_t velocity) {
    setTransmissionState(speed, transmission_state);
    setVelocity(speed, velocity);
  }

  /**
   * @brief Set the semi-major axis accuracy value
   *
   * @param accuracy SemiMajorAxisAccuracy object to set
   * @param value accuracy value
   */
  inline void setSemiMajorAxisAccuracy(SemiMajorAxisAccuracy& accuracy, const uint8_t value) {
    throwIfOutOfRange(value, SemiMajorAxisAccuracy::MIN, SemiMajorAxisAccuracy::MAX, "SemiMajorAxisAccuracy");
    accuracy.value = value;
  }

  /**
   * @brief Set the semi-major axis accuracy of a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to set
   * @param value accuracy value
   */
  inline void setSemiMajorAxisAccuracy(PositionalAccuracy& accuracy, const uint8_t value) {
    setSemiMajorAxisAccuracy(accuracy.semi_major, value);
  }

  /**
   * @brief Set the semi-minor axis accuracy value
   *
   * @param accuracy SemiMinorAxisAccuracy object to set
   * @param value accuracy value
   */
  inline void setSemiMinorAxisAccuracy(SemiMinorAxisAccuracy& accuracy, const uint8_t value) {
    throwIfOutOfRange(value, SemiMinorAxisAccuracy::MIN, SemiMinorAxisAccuracy::MAX, "SemiMinorAxisAccuracy");
    accuracy.value = value;
  }

  /**
   * @brief Set the semi-minor axis accuracy of a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to set
   * @param value accuracy value
   */
  inline void setSemiMinorAxisAccuracy(PositionalAccuracy& accuracy, const uint8_t value) {
    setSemiMinorAxisAccuracy(accuracy.semi_minor, value);
  }

  /**
   * @brief Set the semi-major axis orientation value
   *
   * @param orientation SemiMajorAxisOrientation object to set
   * @param value orientation value
   */
  inline void setSemiMajorAxisOrientation(SemiMajorAxisOrientation& orientation, const uint16_t value) {
    throwIfOutOfRange(value, SemiMajorAxisOrientation::MIN, SemiMajorAxisOrientation::MAX,
                      "SemiMajorAxisOrientation");
    orientation.value = value;
  }

  /**
   * @brief Set the semi-major axis orientation of a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to set
   * @param value orientation value
   */
  inline void setSemiMajorAxisOrientation(PositionalAccuracy& accuracy, const uint16_t value) {
    setSemiMajorAxisOrientation(accuracy.orientation, value);
  }

  /**
   * @brief Set the position confidence value
   *
   * @param confidence PositionConfidence object to set
   * @param value confidence value
   */
  inline void setPositionConfidence(PositionConfidence& confidence, const uint8_t value) {
    throwIfOutOfRange(value, PositionConfidence::UNAVAILABLE, PositionConfidence::A1CM, "PositionConfidence");
    confidence.value = value;
  }

  /**
   * @brief Set the position confidence of a PositionConfidenceSet object
   *
   * @param confidence PositionConfidenceSet object to set
   * @param value confidence value
   */
  inline void setPositionConfidence(PositionConfidenceSet& confidence, const uint8_t value) {
    setPositionConfidence(confidence.pos, value);
  }

  /**
   * @brief Set the elevation confidence value
   *
   * @param confidence ElevationConfidence object to set
   * @param value confidence value
   */
  inline void setElevationConfidence(ElevationConfidence& confidence, const uint8_t value) {
    throwIfOutOfRange(value, ElevationConfidence::UNAVAILABLE, ElevationConfidence::ELEV_000_01,
                      "ElevationConfidence");
    confidence.value = value;
  }

  /**
   * @brief Set the elevation confidence of a PositionConfidenceSet object
   *
   * @param confidence PositionConfidenceSet object to set
   * @param value confidence value
   */
  inline void setElevationConfidence(PositionConfidenceSet& confidence, const uint8_t value) {
    setElevationConfidence(confidence.elevation, value);
  }

  /**
   * @brief Set the heading confidence value
   *
   * @param confidence HeadingConfidenceDSRC object to set
   * @param value confidence value
   */
  inline void setHeadingConfidence(HeadingConfidenceDSRC& confidence, const uint8_t value) {
    throwIfOutOfRange(value, HeadingConfidenceDSRC::UNAVAILABLE, HeadingConfidenceDSRC::PREC0_0125DEG,
                      "HeadingConfidenceDSRC");
    confidence.value = value;
  }

  /**
   * @brief Set the heading confidence of a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to set
   * @param value confidence value
   */
  inline void setHeadingConfidence(SpeedandHeadingandThrottleConfidence& confidence, const uint8_t value) {
    setHeadingConfidence(confidence.heading, value);
  }

  /**
   * @brief Set the speed confidence value
   *
   * @param confidence SpeedConfidenceDSRC object to set
   * @param value confidence value
   */
  inline void setSpeedConfidence(SpeedConfidenceDSRC& confidence, const uint8_t value) {
    throwIfOutOfRange(value, SpeedConfidenceDSRC::UNAVAILABLE, SpeedConfidenceDSRC::PREC0_01MS,
                      "SpeedConfidenceDSRC");
    confidence.value = value;
  }

  /**
   * @brief Set the speed confidence of a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to set
   * @param value confidence value
   */
  inline void setSpeedConfidence(SpeedandHeadingandThrottleConfidence& confidence, const uint8_t value) {
    setSpeedConfidence(confidence.speed, value);
  }

  /**
   * @brief Set the throttle confidence value
   *
   * @param confidence ThrottleConfidence object to set
   * @param value confidence value
   */
  inline void setThrottleConfidence(ThrottleConfidence& confidence, const uint8_t value) {
    throwIfOutOfRange(value, ThrottleConfidence::UNAVAILABLE, ThrottleConfidence::PREC0_5PERCENT,
                      "ThrottleConfidence");
    confidence.value = value;
  }

  /**
   * @brief Set the throttle confidence of a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to set
   * @param value confidence value
   */
  inline void setThrottleConfidence(SpeedandHeadingandThrottleConfidence& confidence, const uint8_t value) {
    setThrottleConfidence(confidence.throttle, value);
  }

  /**
   * @brief Set the DYear value
   *
   * @param year DYear object to set
   * @param value year value
   */
  inline void setYear(DYear& year, const uint16_t value) {
    throwIfOutOfRange(value, DYear::MIN, DYear::MAX, "DYear");
    year.value = value;
  }

  /**
   * @brief Set the year of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value year value
   */
  inline void setYear(DDateTime& date_time, const uint16_t value) {
    setYear(date_time.year, value);
    date_time.year_is_present = true;
  }

  /**
   * @brief Clear the optional year of a DDateTime object
   *
   * @param date_time DDateTime object to update
   */
  inline void clearYear(DDateTime& date_time) {
    date_time.year_is_present = false;
  }

  /**
   * @brief Set the DMonth value
   *
   * @param month DMonth object to set
   * @param value month value
   */
  inline void setMonth(DMonth& month, const uint8_t value) {
    throwIfOutOfRange(value, DMonth::MIN, DMonth::MAX, "DMonth");
    month.value = value;
  }

  /**
   * @brief Set the month of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value month value
   */
  inline void setMonth(DDateTime& date_time, const uint8_t value) {
    setMonth(date_time.month, value);
    date_time.month_is_present = true;
  }

  /**
   * @brief Clear the optional month of a DDateTime object
   *
   * @param date_time DDateTime object to update
   */
  inline void clearMonth(DDateTime& date_time) {
    date_time.month_is_present = false;
  }

  /**
   * @brief Set the DDay value
   *
   * @param day DDay object to set
   * @param value day value
   */
  inline void setDay(DDay& day, const uint8_t value) {
    throwIfOutOfRange(value, DDay::MIN, DDay::MAX, "DDay");
    day.value = value;
  }

  /**
   * @brief Set the day of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value day value
   */
  inline void setDay(DDateTime& date_time, const uint8_t value) {
    setDay(date_time.day, value);
    date_time.day_is_present = true;
  }

  /**
   * @brief Clear the optional day of a DDateTime object
   *
   * @param date_time DDateTime object to update
   */
  inline void clearDay(DDateTime& date_time) {
    date_time.day_is_present = false;
  }

  /**
   * @brief Set the DHour value
   *
   * @param hour DHour object to set
   * @param value hour value
   */
  inline void setHour(DHour& hour, const uint8_t value) {
    throwIfOutOfRange(value, DHour::MIN, DHour::MAX, "DHour");
    hour.value = value;
  }

  /**
   * @brief Set the hour of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value hour value
   */
  inline void setHour(DDateTime& date_time, const uint8_t value) {
    setHour(date_time.hour, value);
    date_time.hour_is_present = true;
  }

  /**
   * @brief Clear the optional hour of a DDateTime object
   *
   * @param date_time DDateTime object to update
   */
  inline void clearHour(DDateTime& date_time) {
    date_time.hour_is_present = false;
  }

  /**
   * @brief Set the DMinute value
   *
   * @param minute DMinute object to set
   * @param value minute value
   */
  inline void setMinute(DMinute& minute, const uint8_t value) {
    throwIfOutOfRange(value, DMinute::MIN, DMinute::MAX, "DMinute");
    minute.value = value;
  }

  /**
   * @brief Set the minute of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value minute value
   */
  inline void setMinute(DDateTime& date_time, const uint8_t value) {
    setMinute(date_time.minute, value);
    date_time.minute_is_present = true;
  }

  /**
   * @brief Clear the optional minute of a DDateTime object
   *
   * @param date_time DDateTime object to update
   */
  inline void clearMinute(DDateTime& date_time) {
    date_time.minute_is_present = false;
  }

  /**
   * @brief Set the DSecond value
   *
   * @param second DSecond object to set
   * @param value second value in milliseconds
   */
  inline void setSecond(DSecond& second, const uint16_t value) {
    throwIfOutOfRange(value, DSecond::MIN, DSecond::MAX, "DSecond");
    second.value = value;
  }

  /**
   * @brief Set the DSecond value
   *
   * @param second DSecond object to set
   * @param value second value in seconds
   */
  inline void setSecond(DSecond& second, const double value) {
    setSecond(second, static_cast<uint16_t>(std::round(value * 1e3)));
  }

  /**
   * @brief Set the second of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value second value in milliseconds
   */
  inline void setSecond(DDateTime& date_time, const uint16_t value) {
    setSecond(date_time.second, value);
    date_time.second_is_present = true;
  }

  /**
   * @brief Set the second of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value second value in seconds
   */
  inline void setSecond(DDateTime& date_time, const double value) {
    setSecond(date_time.second, value);
    date_time.second_is_present = true;
  }

  /**
   * @brief Clear the optional second of a DDateTime object
   *
   * @param date_time DDateTime object to update
   */
  inline void clearSecond(DDateTime& date_time) {
    date_time.second_is_present = false;
  }

  /**
   * @brief Set the DOffset value
   *
   * @param offset DOffset object to set
   * @param value offset value
   */
  inline void setOffset(DOffset& offset, const int16_t value) {
    throwIfOutOfRange(value, DOffset::MIN, DOffset::MAX, "DOffset");
    offset.value = value;
  }

  /**
   * @brief Set the offset of a DDateTime object
   *
   * @param date_time DDateTime object to set
   * @param value offset value
   */
  inline void setOffset(DDateTime& date_time, const int16_t value) {
    setOffset(date_time.offset, value);
    date_time.offset_is_present = true;
  }

  /**
   * @brief Clear the optional offset of a DDateTime object
   *
   * @param date_time DDateTime object to update
   */
  inline void clearOffset(DDateTime& date_time) {
    date_time.offset_is_present = false;
  }

} // namespace access

} // namespace etsi_its_rtcmem_ts_msgs
