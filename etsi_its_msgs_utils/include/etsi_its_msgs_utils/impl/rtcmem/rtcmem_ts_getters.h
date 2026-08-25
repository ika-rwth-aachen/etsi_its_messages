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
 * @file impl/rtcmem/rtcmem_ts_getters.h
 * @brief Getter functions for the ETSI ITS RTCMEM
 */

#pragma once

#include <stdexcept>
#include <vector>

namespace etsi_its_rtcmem_ts_msgs {

namespace access {

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_getters.h>
#include <etsi_its_msgs_utils/impl/checks.h>

  /**
   * @brief Get the protocol version from an ItsPduHeader object
   *
   * @param header ItsPduHeader object to get the protocol version from
   * @return uint8_t the protocol version
   */
  inline uint8_t getProtocolVersion(const ItsPduHeader& header) {
    return header.protocol_version;
  }

  /**
   * @brief Get the message ID from an ItsPduHeader object
   *
   * @param header ItsPduHeader object to get the message ID from
   * @return uint8_t the message ID
   */
  inline uint8_t getMessageID(const ItsPduHeader& header) {
    return header.message_id;
  }

  /**
   * @brief Get the StationID object from an ItsPduHeader object
   *
   * @param header ItsPduHeader object to get the StationID from
   * @return StationID the station ID object
   */
  inline StationID getStationID(const ItsPduHeader& header) {
    return header.station_id;
  }

  /**
   * @brief Get the StationID value from an ItsPduHeader object
   *
   * @param header ItsPduHeader object to get the StationID value from
   * @return uint32_t the station ID value
   */
  inline uint32_t getStationIDValue(const ItsPduHeader& header) {
    return getStationID(header).value;
  }

  /**
   * @brief Get the ItsPduHeader from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the header from
   * @return ItsPduHeader the ITS PDU header
   */
  inline ItsPduHeader getHeader(const RTCMEM& rtcmem) {
    return rtcmem.header;
  }

  /**
   * @brief Get the protocol version from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the protocol version from
   * @return uint8_t the protocol version
   */
  inline uint8_t getProtocolVersion(const RTCMEM& rtcmem) {
    return getProtocolVersion(rtcmem.header);
  }

  /**
   * @brief Get the message ID from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the message ID from
   * @return uint8_t the message ID
   */
  inline uint8_t getMessageID(const RTCMEM& rtcmem) {
    return getMessageID(rtcmem.header);
  }

  /**
   * @brief Get the StationID object from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the StationID from
   * @return StationID the station ID object
   */
  inline StationID getStationID(const RTCMEM& rtcmem) {
    return getStationID(rtcmem.header);
  }

  /**
   * @brief Get the StationID value from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the StationID value from
   * @return uint32_t the station ID value
   */
  inline uint32_t getStationIDValue(const RTCMEM& rtcmem) {
    return getStationIDValue(rtcmem.header);
  }

  /**
   * @brief Get the RTCMcorrections container from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the RTCMcorrections from
   * @return RTCMcorrections the RTCMcorrections object
   */
  inline RTCMcorrections getRTCMcorrections(const RTCMEM& rtcmem) {
    return rtcmem.rtcmc;
  }

  /**
   * @brief Get the MsgCount value from a MsgCount object
   *
   * @param msg_cnt MsgCount object to get the value from
   * @return uint8_t the MsgCount value
   */
  inline uint8_t getMsgCount(const MsgCount& msg_cnt) {
    return msg_cnt.value;
  }

  /**
   * @brief Get the MsgCount value from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the MsgCount value from
   * @return uint8_t the MsgCount value
   */
  inline uint8_t getMsgCount(const RTCMcorrections& rtcm) {
    return getMsgCount(rtcm.msg_cnt);
  }

  /**
   * @brief Get the MsgCount value from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the MsgCount value from
   * @return uint8_t the MsgCount value
   */
  inline uint8_t getMsgCount(const RTCMEM& rtcmem) {
    return getMsgCount(rtcmem.rtcmc);
  }

  /**
   * @brief Get the RTCMRevision object
   *
   * @param rtcm RTCMcorrections object to get the RTCMRevision from
   * @return RTCMRevision the RTCM revision object
   */
  inline RTCMRevision getRTCMRevision(const RTCMcorrections& rtcm) {
    return rtcm.rev;
  }

  /**
   * @brief Get the RTCMRevision object from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the RTCMRevision from
   * @return RTCMRevision the RTCM revision object
   */
  inline RTCMRevision getRTCMRevision(const RTCMEM& rtcmem) {
    return getRTCMRevision(rtcmem.rtcmc);
  }

  /**
   * @brief Get the RTCMRevision value from a RTCMRevision object
   *
   * @param rev RTCMRevision object to get the value from
   * @return uint8_t the RTCM revision value
   */
  inline uint8_t getRTCMRevisionValue(const RTCMRevision& rev) {
    return rev.value;
  }

  /**
   * @brief Get the RTCMRevision value from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the RTCMRevision value from
   * @return uint8_t the RTCM revision value
   */
  inline uint8_t getRTCMRevisionValue(const RTCMcorrections& rtcm) {
    return getRTCMRevisionValue(rtcm.rev);
  }

  /**
   * @brief Get the RTCMRevision value from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the RTCMRevision value from
   * @return uint8_t the RTCM revision value
   */
  inline uint8_t getRTCMRevisionValue(const RTCMEM& rtcmem) {
    return getRTCMRevisionValue(rtcmem.rtcmc);
  }

  /**
   * @brief Check whether a MinuteOfTheYear is present in a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to check
   * @return bool true if the timestamp is present
   */
  inline bool hasMinuteOfTheYear(const RTCMcorrections& rtcm) {
    return rtcm.time_stamp_is_present;
  }

  /**
   * @brief Check whether a MinuteOfTheYear is present in a RTCMEM object
   *
   * @param rtcmem RTCMEM object to check
   * @return bool true if the timestamp is present
   */
  inline bool hasMinuteOfTheYear(const RTCMEM& rtcmem) {
    return hasMinuteOfTheYear(rtcmem.rtcmc);
  }

  /**
   * @brief Get the MinuteOfTheYear object from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the MinuteOfTheYear from
   * @return MinuteOfTheYear the timestamp object
   */
  inline MinuteOfTheYear getMinuteOfTheYear(const RTCMcorrections& rtcm) {
    throwIfNotPresent(rtcm.time_stamp_is_present, "rtcm.time_stamp");
    return rtcm.time_stamp;
  }

  /**
   * @brief Get the MinuteOfTheYear object from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the MinuteOfTheYear from
   * @return MinuteOfTheYear the timestamp object
   */
  inline MinuteOfTheYear getMinuteOfTheYear(const RTCMEM& rtcmem) {
    return getMinuteOfTheYear(rtcmem.rtcmc);
  }

  /**
   * @brief Get the MinuteOfTheYear value from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the value from
   * @return uint32_t the MinuteOfTheYear value
   */
  inline uint32_t getMinuteOfTheYearValue(const RTCMcorrections& rtcm) {
    return getMinuteOfTheYear(rtcm).value;
  }

  /**
   * @brief Get the MinuteOfTheYear value from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the value from
   * @return uint32_t the MinuteOfTheYear value
   */
  inline uint32_t getMinuteOfTheYearValue(const RTCMEM& rtcmem) {
    return getMinuteOfTheYearValue(rtcmem.rtcmc);
  }

  /**
   * @brief Check whether an anchor point is present in a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to check
   * @return bool true if the anchor point is present
   */
  inline bool hasAnchorPoint(const RTCMcorrections& rtcm) {
    return rtcm.anchor_point_is_present;
  }

  /**
   * @brief Check whether an anchor point is present in a RTCMEM object
   *
   * @param rtcmem RTCMEM object to check
   * @return bool true if the anchor point is present
   */
  inline bool hasAnchorPoint(const RTCMEM& rtcmem) {
    return hasAnchorPoint(rtcmem.rtcmc);
  }

  /**
   * @brief Get the anchor point from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the anchor point from
   * @return FullPositionVector the anchor point
   */
  inline FullPositionVector getAnchorPoint(const RTCMcorrections& rtcm) {
    throwIfNotPresent(rtcm.anchor_point_is_present, "rtcm.anchor_point");
    return rtcm.anchor_point;
  }

  /**
   * @brief Get the anchor point from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the anchor point from
   * @return FullPositionVector the anchor point
   */
  inline FullPositionVector getAnchorPoint(const RTCMEM& rtcmem) {
    return getAnchorPoint(rtcmem.rtcmc);
  }

  /**
   * @brief Check whether a RTCMheader is present in a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to check
   * @return bool true if the RTCMheader is present
   */
  inline bool hasRTCMHeader(const RTCMcorrections& rtcm) {
    return rtcm.rtcm_header_is_present;
  }

  /**
   * @brief Check whether a RTCMheader is present in a RTCMEM object
   *
   * @param rtcmem RTCMEM object to check
   * @return bool true if the RTCMheader is present
   */
  inline bool hasRTCMHeader(const RTCMEM& rtcmem) {
    return hasRTCMHeader(rtcmem.rtcmc);
  }

  /**
   * @brief Get the RTCMheader from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the RTCMheader from
   * @return RTCMheader the RTCM header
   */
  inline RTCMheader getRTCMHeader(const RTCMcorrections& rtcm) {
    throwIfNotPresent(rtcm.rtcm_header_is_present, "rtcm.rtcm_header");
    return rtcm.rtcm_header;
  }

  /**
   * @brief Get the RTCMheader from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the RTCMheader from
   * @return RTCMheader the RTCM header
   */
  inline RTCMheader getRTCMHeader(const RTCMEM& rtcmem) {
    return getRTCMHeader(rtcmem.rtcmc);
  }

  /**
   * @brief Get the RTCMmessageList from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the RTCMmessageList from
   * @return RTCMmessageList the RTCM message list
   */
  inline RTCMmessageList getRTCMmessageList(const RTCMcorrections& rtcm) {
    return rtcm.msgs;
  }

  /**
   * @brief Get the RTCMmessageList from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the RTCMmessageList from
   * @return RTCMmessageList the RTCM message list
   */
  inline RTCMmessageList getRTCMmessageList(const RTCMEM& rtcmem) {
    return getRTCMmessageList(rtcmem.rtcmc);
  }

  /**
   * @brief Get the RTCM messages from a RTCMmessageList object
   *
   * @param msgs RTCMmessageList object to get the messages from
   * @return std::vector<RTCMmessage> the RTCM messages
   */
  inline std::vector<RTCMmessage> getRTCMmessages(const RTCMmessageList& msgs) {
    return msgs.array;
  }

  /**
   * @brief Get the RTCM messages from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the messages from
   * @return std::vector<RTCMmessage> the RTCM messages
   */
  inline std::vector<RTCMmessage> getRTCMmessages(const RTCMcorrections& rtcm) {
    return getRTCMmessages(rtcm.msgs);
  }

  /**
   * @brief Get the RTCM messages from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the messages from
   * @return std::vector<RTCMmessage> the RTCM messages
   */
  inline std::vector<RTCMmessage> getRTCMmessages(const RTCMEM& rtcmem) {
    return getRTCMmessages(rtcmem.rtcmc);
  }

  /**
   * @brief Get the number of RTCM messages from a RTCMmessageList object
   *
   * @param msgs RTCMmessageList object to get the message count from
   * @return std::size_t the number of RTCM messages
   */
  inline std::size_t getRTCMmessageCount(const RTCMmessageList& msgs) {
    return msgs.array.size();
  }

  /**
   * @brief Get the number of RTCM messages from a RTCMcorrections object
   *
   * @param rtcm RTCMcorrections object to get the message count from
   * @return std::size_t the number of RTCM messages
   */
  inline std::size_t getRTCMmessageCount(const RTCMcorrections& rtcm) {
    return getRTCMmessageCount(rtcm.msgs);
  }

  /**
   * @brief Get the number of RTCM messages from a RTCMEM object
   *
   * @param rtcmem RTCMEM object to get the message count from
   * @return std::size_t the number of RTCM messages
   */
  inline std::size_t getRTCMmessageCount(const RTCMEM& rtcmem) {
    return getRTCMmessageCount(rtcmem.rtcmc);
  }

  /**
   * @brief Get a RTCMmessage from a RTCMmessageList object by index
   *
   * @param msgs RTCMmessageList object to get the message from
   * @param index zero-based message index
   * @return RTCMmessage the RTCM message at the given index
   */
  inline RTCMmessage getRTCMmessage(const RTCMmessageList& msgs, const std::size_t index) {
    if (index >= msgs.array.size()) {
      throw std::out_of_range("RTCMmessageList index out of range.");
    }
    return msgs.array[index];
  }

  /**
   * @brief Get a RTCMmessage from a RTCMcorrections object by index
   *
   * @param rtcm RTCMcorrections object to get the message from
   * @param index zero-based message index
   * @return RTCMmessage the RTCM message at the given index
   */
  inline RTCMmessage getRTCMmessage(const RTCMcorrections& rtcm, const std::size_t index) {
    return getRTCMmessage(rtcm.msgs, index);
  }

  /**
   * @brief Get a RTCMmessage from a RTCMEM object by index
   *
   * @param rtcmem RTCMEM object to get the message from
   * @param index zero-based message index
   * @return RTCMmessage the RTCM message at the given index
   */
  inline RTCMmessage getRTCMmessage(const RTCMEM& rtcmem, const std::size_t index) {
    return getRTCMmessage(rtcmem.rtcmc, index);
  }

  /**
   * @brief Get the raw bytes from a RTCMmessage object
   *
   * @param msg RTCMmessage object to get the bytes from
   * @return std::vector<uint8_t> the RTCM payload bytes
   */
  inline std::vector<uint8_t> getRTCMmessageValue(const RTCMmessage& msg) {
    return msg.value;
  }

  /**
   * @brief Get the size of a RTCMmessage object in bytes
   *
   * @param msg RTCMmessage object to get the size from
   * @return std::size_t the message size in bytes
   */
  inline std::size_t getRTCMmessageSize(const RTCMmessage& msg) {
    return msg.value.size();
  }

  /**
   * @brief Get the GNSSstatus object from a RTCMheader object
   *
   * @param header RTCMheader object to get the GNSSstatus from
   * @return GNSSstatus the GNSSstatus object
   */
  inline GNSSstatus getGNSSstatus(const RTCMheader& header) {
    return header.status;
  }

  /**
   * @brief Get the GNSSstatus as a bool vector
   *
   * @param status GNSSstatus object to get the bool vector from
   * @return std::vector<bool> the GNSSstatus bits
   */
  inline std::vector<bool> getGNSSstatus(const GNSSstatus& status) {
    return getBitString(status.value, status.bits_unused);
  }

  /**
   * @brief Get the GNSSstatus as a bool vector from a RTCMheader object
   *
   * @param header RTCMheader object to get the bool vector from
   * @return std::vector<bool> the GNSSstatus bits
   */
  inline std::vector<bool> getGNSSstatusBits(const RTCMheader& header) {
    return getGNSSstatus(header.status);
  }

  /**
   * @brief Get the AntennaOffsetSet from a RTCMheader object
   *
   * @param header RTCMheader object to get the AntennaOffsetSet from
   * @return AntennaOffsetSet the antenna offset set
   */
  inline AntennaOffsetSet getAntennaOffsetSet(const RTCMheader& header) {
    return header.offset_set;
  }

  /**
   * @brief Get a GNSSstatus bit by index
   *
   * @param status GNSSstatus object to get the bit from
   * @param bit_index bit index to read
   * @return bool true if the bit is set
   */
  inline bool getGNSSstatusBit(const GNSSstatus& status, const uint8_t bit_index) {
    throwIfOutOfRange(bit_index, static_cast<uint8_t>(0), static_cast<uint8_t>(GNSSstatus::SIZE_BITS - 1), "GNSSstatus bit index");

    if (status.value.empty()) {
      return false;
    }

    const uint8_t mask = static_cast<uint8_t>(1u << bit_index);
    return (status.value[0] & mask) != 0;
  }

  /**
   * @brief Get the unavailable flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if unavailable is set
   */
  inline bool getUnavailableFlag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_UNAVAILABLE);
  }

  /**
   * @brief Get the healthy flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if isHealthy is set
   */
  inline bool getIsHealthyFlag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_IS_HEALTHY);
  }

  /**
   * @brief Get the monitored flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if isMonitored is set
   */
  inline bool getIsMonitoredFlag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_IS_MONITORED);
  }

  /**
   * @brief Get the base station type flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if baseStationType is set
   */
  inline bool getBaseStationTypeFlag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_BASE_STATION_TYPE);
  }

  /**
   * @brief Get the aPDOPofUnder5 flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if aPDOPofUnder5 is set
   */
  inline bool getAPDOPofUnder5Flag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_A_PDO_POF_UNDER5);
  }

  /**
   * @brief Get the inViewOfUnder5 flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if inViewOfUnder5 is set
   */
  inline bool getInViewOfUnder5Flag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_IN_VIEW_OF_UNDER5);
  }

  /**
   * @brief Get the localCorrectionsPresent flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if localCorrectionsPresent is set
   */
  inline bool getLocalCorrectionsPresentFlag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_LOCAL_CORRECTIONS_PRESENT);
  }

  /**
   * @brief Get the networkCorrectionsPresent flag from a GNSSstatus object
   *
   * @param status GNSSstatus object to get the flag from
   * @return bool true if networkCorrectionsPresent is set
   */
  inline bool getNetworkCorrectionsPresentFlag(const GNSSstatus& status) {
    return getGNSSstatusBit(status, GNSSstatus::BIT_INDEX_NETWORK_CORRECTIONS_PRESENT);
  }

  /**
   * @brief Get the antenna offset in x direction
   *
   * @param offset AntennaOffsetSet object to get the x offset from
   * @return int16_t the antenna offset x value
   */
  inline int16_t getAntennaOffsetXValue(const AntennaOffsetSet& offset) {
    return offset.ant_offset_x.value;
  }

  /**
   * @brief Get the antenna offset in y direction
   *
   * @param offset AntennaOffsetSet object to get the y offset from
   * @return int16_t the antenna offset y value
   */
  inline int16_t getAntennaOffsetYValue(const AntennaOffsetSet& offset) {
    return offset.ant_offset_y.value;
  }

  /**
   * @brief Get the antenna offset in z direction
   *
   * @param offset AntennaOffsetSet object to get the z offset from
   * @return int16_t the antenna offset z value
   */
  inline int16_t getAntennaOffsetZValue(const AntennaOffsetSet& offset) {
    return offset.ant_offset_z.value;
  }

  /**
   * @brief Get the longitude value from a Longitude object
   *
   * @param longitude Longitude object to get the value from
   * @return double the longitude in degree
   */
  inline double getLongitude(const Longitude& longitude) {
    return static_cast<double>(longitude.value) * 1e-7;
  }

  /**
   * @brief Get the latitude value from a Latitude object
   *
   * @param latitude Latitude object to get the value from
   * @return double the latitude in degree
   */
  inline double getLatitude(const Latitude& latitude) {
    return static_cast<double>(latitude.value) * 1e-7;
  }

  /**
   * @brief Get the elevation value from an Elevation object
   *
   * @param elevation Elevation object to get the value from
   * @return double the elevation in meter
   */
  inline double getElevation(const Elevation& elevation) {
    return static_cast<double>(elevation.value) * 1e-1;
  }

  /**
   * @brief Get the heading value from a HeadingDSRC object
   *
   * @param heading HeadingDSRC object to get the value from
   * @return uint16_t the heading value
   */
  inline uint16_t getHeadingValue(const HeadingDSRC& heading) {
    return heading.value;
  }

  /**
   * @brief Get the utcTime presence flag from a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if utcTime is present
   */
  inline bool hasUTCTime(const FullPositionVector& pos) {
    return pos.utc_time_is_present;
  }

  /**
   * @brief Get the utcTime object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the utcTime from
   * @return DDateTime the utcTime object
   */
  inline DDateTime getUTCTime(const FullPositionVector& pos) {
    throwIfNotPresent(pos.utc_time_is_present, "pos.utc_time");
    return pos.utc_time;
  }

  /**
   * @brief Get the raw longitude value from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the value from
   * @return int32_t the raw longitude value
   */
  inline int32_t getLongitudeValue(const FullPositionVector& pos) {
    return pos.lon.value;
  }

  /**
   * @brief Get the longitude in degree from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the longitude from
   * @return double the longitude in degree
   */
  inline double getLongitude(const FullPositionVector& pos) {
    return getLongitude(pos.lon);
  }

  /**
   * @brief Get the raw latitude value from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the value from
   * @return int32_t the raw latitude value
   */
  inline int32_t getLatitudeValue(const FullPositionVector& pos) {
    return pos.lat.value;
  }

  /**
   * @brief Get the latitude in degree from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the latitude from
   * @return double the latitude in degree
   */
  inline double getLatitude(const FullPositionVector& pos) {
    return getLatitude(pos.lat);
  }

  /**
   * @brief Check whether elevation is present in a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if elevation is present
   */
  inline bool hasElevation(const FullPositionVector& pos) {
    return pos.elevation_is_present;
  }

  /**
   * @brief Get the Elevation object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the elevation from
   * @return Elevation the elevation object
   */
  inline Elevation getElevationObject(const FullPositionVector& pos) {
    throwIfNotPresent(pos.elevation_is_present, "pos.elevation");
    return pos.elevation;
  }

  /**
   * @brief Get the elevation in meter from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the elevation from
   * @return double the elevation in meter
   */
  inline double getElevation(const FullPositionVector& pos) {
    return getElevation(getElevationObject(pos));
  }

  /**
   * @brief Check whether heading is present in a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if heading is present
   */
  inline bool hasHeading(const FullPositionVector& pos) {
    return pos.heading_is_present;
  }

  /**
   * @brief Get the HeadingDSRC object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the heading from
   * @return HeadingDSRC the heading object
   */
  inline HeadingDSRC getHeading(const FullPositionVector& pos) {
    throwIfNotPresent(pos.heading_is_present, "pos.heading");
    return pos.heading;
  }

  /**
   * @brief Check whether speed is present in a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if speed is present
   */
  inline bool hasSpeed(const FullPositionVector& pos) {
    return pos.speed_is_present;
  }

  /**
   * @brief Get the TransmissionAndSpeed object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the speed from
   * @return TransmissionAndSpeed the speed object
   */
  inline TransmissionAndSpeed getSpeed(const FullPositionVector& pos) {
    throwIfNotPresent(pos.speed_is_present, "pos.speed");
    return pos.speed;
  }

  /**
   * @brief Check whether positional accuracy is present in a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if positional accuracy is present
   */
  inline bool hasPositionalAccuracy(const FullPositionVector& pos) {
    return pos.pos_accuracy_is_present;
  }

  /**
   * @brief Get the PositionalAccuracy object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the positional accuracy from
   * @return PositionalAccuracy the positional accuracy object
   */
  inline PositionalAccuracy getPositionalAccuracy(const FullPositionVector& pos) {
    throwIfNotPresent(pos.pos_accuracy_is_present, "pos.pos_accuracy");
    return pos.pos_accuracy;
  }

  /**
   * @brief Check whether time confidence is present in a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if time confidence is present
   */
  inline bool hasTimeConfidence(const FullPositionVector& pos) {
    return pos.time_confidence_is_present;
  }

  /**
   * @brief Get the TimeConfidence object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the time confidence from
   * @return TimeConfidence the time confidence object
   */
  inline TimeConfidence getTimeConfidence(const FullPositionVector& pos) {
    throwIfNotPresent(pos.time_confidence_is_present, "pos.time_confidence");
    return pos.time_confidence;
  }

  /**
   * @brief Get the TimeConfidence value from a TimeConfidence object
   *
   * @param time_confidence TimeConfidence object to get the value from
   * @return uint8_t the time confidence value
   */
  inline uint8_t getTimeConfidenceValue(const TimeConfidence& time_confidence) {
    return time_confidence.value;
  }

  /**
   * @brief Check whether position confidence is present in a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if position confidence is present
   */
  inline bool hasPositionConfidence(const FullPositionVector& pos) {
    return pos.pos_confidence_is_present;
  }

  /**
   * @brief Get the PositionConfidenceSet object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the position confidence from
   * @return PositionConfidenceSet the position confidence object
   */
  inline PositionConfidenceSet getPositionConfidence(const FullPositionVector& pos) {
    throwIfNotPresent(pos.pos_confidence_is_present, "pos.pos_confidence");
    return pos.pos_confidence;
  }

  /**
   * @brief Check whether speed confidence is present in a FullPositionVector object
   *
   * @param pos FullPositionVector object to check
   * @return bool true if speed confidence is present
   */
  inline bool hasSpeedConfidence(const FullPositionVector& pos) {
    return pos.speed_confidence_is_present;
  }

  /**
   * @brief Get the SpeedandHeadingandThrottleConfidence object from a FullPositionVector object
   *
   * @param pos FullPositionVector object to get the speed confidence from
   * @return SpeedandHeadingandThrottleConfidence the speed confidence object
   */
  inline SpeedandHeadingandThrottleConfidence getSpeedConfidence(const FullPositionVector& pos) {
    throwIfNotPresent(pos.speed_confidence_is_present, "pos.speed_confidence");
    return pos.speed_confidence;
  }

  /**
   * @brief Get the transmission state object from a TransmissionAndSpeed object
   *
   * @param speed TransmissionAndSpeed object to get the transmission state from
   * @return TransmissionState the transmission state object
   */
  inline TransmissionState getTransmissionState(const TransmissionAndSpeed& speed) {
    return speed.transmisson;
  }

  /**
   * @brief Get the transmission state value from a TransmissionAndSpeed object
   *
   * @param speed TransmissionAndSpeed object to get the transmission state value from
   * @return uint8_t the transmission state value
   */
  inline uint8_t getTransmissionStateValue(const TransmissionAndSpeed& speed) {
    return getTransmissionState(speed).value;
  }

  /**
   * @brief Get the Velocity object from a TransmissionAndSpeed object
   *
   * @param speed TransmissionAndSpeed object to get the Velocity from
   * @return Velocity the velocity object
   */
  inline Velocity getVelocity(const TransmissionAndSpeed& speed) {
    return speed.speed;
  }

  /**
   * @brief Get the Velocity value from a TransmissionAndSpeed object
   *
   * @param speed TransmissionAndSpeed object to get the Velocity value from
   * @return uint16_t the velocity value
   */
  inline uint16_t getVelocityValue(const TransmissionAndSpeed& speed) {
    return getVelocity(speed).value;
  }

  /**
   * @brief Get the semi-major axis accuracy object from a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to get the semi-major axis accuracy from
   * @return SemiMajorAxisAccuracy the semi-major axis accuracy object
   */
  inline SemiMajorAxisAccuracy getSemiMajorAxisAccuracy(const PositionalAccuracy& accuracy) {
    return accuracy.semi_major;
  }

  /**
   * @brief Get the semi-major axis accuracy value from a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to get the value from
   * @return uint8_t the semi-major axis accuracy value
   */
  inline uint8_t getSemiMajorAxisAccuracyValue(const PositionalAccuracy& accuracy) {
    return getSemiMajorAxisAccuracy(accuracy).value;
  }

  /**
   * @brief Get the semi-minor axis accuracy object from a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to get the semi-minor axis accuracy from
   * @return SemiMinorAxisAccuracy the semi-minor axis accuracy object
   */
  inline SemiMinorAxisAccuracy getSemiMinorAxisAccuracy(const PositionalAccuracy& accuracy) {
    return accuracy.semi_minor;
  }

  /**
   * @brief Get the semi-minor axis accuracy value from a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to get the value from
   * @return uint8_t the semi-minor axis accuracy value
   */
  inline uint8_t getSemiMinorAxisAccuracyValue(const PositionalAccuracy& accuracy) {
    return getSemiMinorAxisAccuracy(accuracy).value;
  }

  /**
   * @brief Get the semi-major axis orientation object from a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to get the orientation from
   * @return SemiMajorAxisOrientation the orientation object
   */
  inline SemiMajorAxisOrientation getSemiMajorAxisOrientation(const PositionalAccuracy& accuracy) {
    return accuracy.orientation;
  }

  /**
   * @brief Get the semi-major axis orientation value from a PositionalAccuracy object
   *
   * @param accuracy PositionalAccuracy object to get the value from
   * @return uint16_t the orientation value
   */
  inline uint16_t getSemiMajorAxisOrientationValue(const PositionalAccuracy& accuracy) {
    return getSemiMajorAxisOrientation(accuracy).value;
  }

  /**
   * @brief Get the PositionConfidence object from a PositionConfidenceSet object
   *
   * @param confidence PositionConfidenceSet object to get the position confidence from
   * @return PositionConfidence the position confidence object
   */
  inline PositionConfidence getPositionConfidence(const PositionConfidenceSet& confidence) {
    return confidence.pos;
  }

  /**
   * @brief Get the PositionConfidence value from a PositionConfidenceSet object
   *
   * @param confidence PositionConfidenceSet object to get the value from
   * @return uint8_t the position confidence value
   */
  inline uint8_t getPositionConfidenceValue(const PositionConfidenceSet& confidence) {
    return getPositionConfidence(confidence).value;
  }

  /**
   * @brief Get the ElevationConfidence object from a PositionConfidenceSet object
   *
   * @param confidence PositionConfidenceSet object to get the elevation confidence from
   * @return ElevationConfidence the elevation confidence object
   */
  inline ElevationConfidence getElevationConfidence(const PositionConfidenceSet& confidence) {
    return confidence.elevation;
  }

  /**
   * @brief Get the ElevationConfidence value from a PositionConfidenceSet object
   *
   * @param confidence PositionConfidenceSet object to get the value from
   * @return uint8_t the elevation confidence value
   */
  inline uint8_t getElevationConfidenceValue(const PositionConfidenceSet& confidence) {
    return getElevationConfidence(confidence).value;
  }

  /**
   * @brief Get the HeadingConfidenceDSRC object from a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to get the heading confidence from
   * @return HeadingConfidenceDSRC the heading confidence object
   */
  inline HeadingConfidenceDSRC getHeadingConfidence(const SpeedandHeadingandThrottleConfidence& confidence) {
    return confidence.heading;
  }

  /**
   * @brief Get the HeadingConfidenceDSRC value from a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to get the value from
   * @return uint8_t the heading confidence value
   */
  inline uint8_t getHeadingConfidenceValue(const SpeedandHeadingandThrottleConfidence& confidence) {
    return getHeadingConfidence(confidence).value;
  }

  /**
   * @brief Get the SpeedConfidenceDSRC object from a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to get the speed confidence from
   * @return SpeedConfidenceDSRC the speed confidence object
   */
  inline SpeedConfidenceDSRC getSpeedConfidenceDSRC(const SpeedandHeadingandThrottleConfidence& confidence) {
    return confidence.speed;
  }

  /**
   * @brief Get the SpeedConfidenceDSRC value from a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to get the value from
   * @return uint8_t the speed confidence value
   */
  inline uint8_t getSpeedConfidenceValue(const SpeedandHeadingandThrottleConfidence& confidence) {
    return getSpeedConfidenceDSRC(confidence).value;
  }

  /**
   * @brief Get the ThrottleConfidence object from a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to get the throttle confidence from
   * @return ThrottleConfidence the throttle confidence object
   */
  inline ThrottleConfidence getThrottleConfidence(const SpeedandHeadingandThrottleConfidence& confidence) {
    return confidence.throttle;
  }

  /**
   * @brief Get the ThrottleConfidence value from a SpeedandHeadingandThrottleConfidence object
   *
   * @param confidence SpeedandHeadingandThrottleConfidence object to get the value from
   * @return uint8_t the throttle confidence value
   */
  inline uint8_t getThrottleConfidenceValue(const SpeedandHeadingandThrottleConfidence& confidence) {
    return getThrottleConfidence(confidence).value;
  }

  /**
   * @brief Check whether year is present in a DDateTime object
   *
   * @param date_time DDateTime object to check
   * @return bool true if year is present
   */
  inline bool hasYear(const DDateTime& date_time) {
    return date_time.year_is_present;
  }

  /**
   * @brief Get the DYear object from a DDateTime object
   *
   * @param date_time DDateTime object to get the year from
   * @return DYear the year object
   */
  inline DYear getYear(const DDateTime& date_time) {
    throwIfNotPresent(date_time.year_is_present, "date_time.year");
    return date_time.year;
  }

  /**
   * @brief Get the DYear value from a DDateTime object
   *
   * @param date_time DDateTime object to get the year value from
   * @return uint16_t the year value
   */
  inline uint16_t getYearValue(const DDateTime& date_time) {
    return getYear(date_time).value;
  }

  /**
   * @brief Check whether month is present in a DDateTime object
   *
   * @param date_time DDateTime object to check
   * @return bool true if month is present
   */
  inline bool hasMonth(const DDateTime& date_time) {
    return date_time.month_is_present;
  }

  /**
   * @brief Get the DMonth object from a DDateTime object
   *
   * @param date_time DDateTime object to get the month from
   * @return DMonth the month object
   */
  inline DMonth getMonth(const DDateTime& date_time) {
    throwIfNotPresent(date_time.month_is_present, "date_time.month");
    return date_time.month;
  }

  /**
   * @brief Get the DMonth value from a DDateTime object
   *
   * @param date_time DDateTime object to get the month value from
   * @return uint8_t the month value
   */
  inline uint8_t getMonthValue(const DDateTime& date_time) {
    return getMonth(date_time).value;
  }

  /**
   * @brief Check whether day is present in a DDateTime object
   *
   * @param date_time DDateTime object to check
   * @return bool true if day is present
   */
  inline bool hasDay(const DDateTime& date_time) {
    return date_time.day_is_present;
  }

  /**
   * @brief Get the DDay object from a DDateTime object
   *
   * @param date_time DDateTime object to get the day from
   * @return DDay the day object
   */
  inline DDay getDay(const DDateTime& date_time) {
    throwIfNotPresent(date_time.day_is_present, "date_time.day");
    return date_time.day;
  }

  /**
   * @brief Get the DDay value from a DDateTime object
   *
   * @param date_time DDateTime object to get the day value from
   * @return uint8_t the day value
   */
  inline uint8_t getDayValue(const DDateTime& date_time) {
    return getDay(date_time).value;
  }

  /**
   * @brief Check whether hour is present in a DDateTime object
   *
   * @param date_time DDateTime object to check
   * @return bool true if hour is present
   */
  inline bool hasHour(const DDateTime& date_time) {
    return date_time.hour_is_present;
  }

  /**
   * @brief Get the DHour object from a DDateTime object
   *
   * @param date_time DDateTime object to get the hour from
   * @return DHour the hour object
   */
  inline DHour getHour(const DDateTime& date_time) {
    throwIfNotPresent(date_time.hour_is_present, "date_time.hour");
    return date_time.hour;
  }

  /**
   * @brief Get the DHour value from a DDateTime object
   *
   * @param date_time DDateTime object to get the hour value from
   * @return uint8_t the hour value
   */
  inline uint8_t getHourValue(const DDateTime& date_time) {
    return getHour(date_time).value;
  }

  /**
   * @brief Check whether minute is present in a DDateTime object
   *
   * @param date_time DDateTime object to check
   * @return bool true if minute is present
   */
  inline bool hasMinute(const DDateTime& date_time) {
    return date_time.minute_is_present;
  }

  /**
   * @brief Get the DMinute object from a DDateTime object
   *
   * @param date_time DDateTime object to get the minute from
   * @return DMinute the minute object
   */
  inline DMinute getMinute(const DDateTime& date_time) {
    throwIfNotPresent(date_time.minute_is_present, "date_time.minute");
    return date_time.minute;
  }

  /**
   * @brief Get the DMinute value from a DDateTime object
   *
   * @param date_time DDateTime object to get the minute value from
   * @return uint8_t the minute value
   */
  inline uint8_t getMinuteValue(const DDateTime& date_time) {
    return getMinute(date_time).value;
  }

  /**
   * @brief Check whether second is present in a DDateTime object
   *
   * @param date_time DDateTime object to check
   * @return bool true if second is present
   */
  inline bool hasSecond(const DDateTime& date_time) {
    return date_time.second_is_present;
  }

  /**
   * @brief Get the DSecond object from a DDateTime object
   *
   * @param date_time DDateTime object to get the second from
   * @return DSecond the second object
   */
  inline DSecond getSecond(const DDateTime& date_time) {
    throwIfNotPresent(date_time.second_is_present, "date_time.second");
    return date_time.second;
  }

  /**
   * @brief Get the DSecond value from a DDateTime object
   *
   * @param date_time DDateTime object to get the second value from
   * @return double the second value in seconds
   */
  inline double getSecondValue(const DDateTime& date_time) {
    return static_cast<double>(getSecond(date_time).value) * 1e-3;
  }

  /**
   * @brief Check whether offset is present in a DDateTime object
   *
   * @param date_time DDateTime object to check
   * @return bool true if offset is present
   */
  inline bool hasOffset(const DDateTime& date_time) {
    return date_time.offset_is_present;
  }

  /**
   * @brief Get the DOffset object from a DDateTime object
   *
   * @param date_time DDateTime object to get the offset from
   * @return DOffset the offset object
   */
  inline DOffset getOffset(const DDateTime& date_time) {
    throwIfNotPresent(date_time.offset_is_present, "date_time.offset");
    return date_time.offset;
  }

  /**
   * @brief Get the DOffset value from a DDateTime object
   *
   * @param date_time DDateTime object to get the offset value from
   * @return int16_t the offset value
   */
  inline int16_t getOffsetValue(const DDateTime& date_time) {
    return getOffset(date_time).value;
  }

} // namespace access

} // namespace etsi_its_rtcmem_ts_msgs
