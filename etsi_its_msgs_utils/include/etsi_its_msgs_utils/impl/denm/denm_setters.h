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
 * @file impl/denm/denm_setters.h
 * @brief Setter functions for the ETSI ITS DENM (EN)
 */

#pragma once

#include <etsi_its_msgs_utils/impl/constants.h>

namespace etsi_its_denm_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v1-3-1_setters.h>

  /**
   * @brief Set the ItsPduHeader-object for a DENM
   *
   * @param denm DENM-Message to set the ItsPduHeader
   * @param station_id
   * @param protocol_version
   */
  inline void setItsPduHeader(DENM& denm, const uint32_t station_id, const uint8_t protocol_version = 0){
    setItsPduHeader(denm.header, ItsPduHeader::MESSAGE_ID_DENM, station_id, protocol_version);
  }

  /**
   * @brief Set the ReferenceTime-value
   * 
   * @param denm DENM to set the ReferenceTime-Value for
   * @param unix_nanosecs Timestamp in unix-nanoseconds to set the ReferenceTime-Value from
   * @param n_leap_seconds Number of leap seconds since 2004 for the given timestamp  (Default: etsi_its_msgs::N_LEAP_SECONDS)
   */
  inline void setReferenceTime(DENM& denm, const uint64_t unix_nanosecs, const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second){
    TimestampIts t_its;
    setTimestampITS(t_its, unix_nanosecs, n_leap_seconds);
    throwIfOutOfRange(t_its.value, TimestampIts::MIN, TimestampIts::MAX, "TimestampIts");
    denm.denm.management.reference_time = t_its;
  }

  /**
   * @brief Set the StationType for a DENM
   *
   * @param denm DENM-Message to set the station_type value
   * @param value station_type value to set
   */
  inline void setStationType(DENM& denm, const int value){
    setStationType(denm.denm.management.station_type, value);
  }

  /**
   * @brief Set the ReferencePosition for a DENM
   *
   * Altitude is set to UNAVAILABLE
   *
   * @param denm DENM to set the ReferencePosition
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   */
  inline void setReferencePosition(DENM& denm, const double latitude, const double longitude)
  {
    setReferencePosition(denm.denm.management.event_position, latitude, longitude);
  }

  /**
   * @brief Set the ReferencePosition for a DENM
   *
   * @param denm DENM to set the ReferencePosition
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   * @param altitude Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setReferencePosition(DENM& denm, const double latitude, const double longitude, const double altitude)
  {
    setReferencePosition(denm.denm.management.event_position, latitude, longitude, altitude);
  }

  /**
   * @brief Set the IsHeadingPresent object for DENM
   * 
   * @param denm DENM to set IsHeadingPresent
   * @param presence_of_heading IsHeadingPresent-Value (true or false)
   */
  inline void setIsHeadingPresent(DENM& denm, bool presence_of_heading){
    if(denm.denm.location_is_present){
      denm.denm.location.event_position_heading_is_present = presence_of_heading;
      }
    else{
      throw std::invalid_argument("LocationContainer is not present!");
    }
  }

    /**
   * @brief Set the Heading for a DENM
   *
   * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
   * HeadingConfidence is set to UNAVAILABLE
   *
   * @param denm DENM to set the ReferencePosition
   * @param value Heading value in degree as decimal number
   */
  inline void setHeading(DENM& denm, const double heading_val){
    if(denm.denm.location_is_present){
      setHeading(denm.denm.location.event_position_heading, heading_val);
      setIsHeadingPresent(denm, true);
      }
    else{
      throw std::invalid_argument("LocationContainer is not present!");
    }
  }

  /**
   * @brief Set the IsSpeedPresent object for DENM
   * 
   * @param denm DENM to set IsSpeedPresent
   * @param presence_of_heading IsSpeedPresent-Value (true or false)
   */
  inline void setIsSpeedPresent(DENM& denm, bool presence_of_speed){
    if(denm.denm.location_is_present){
      denm.denm.location.event_speed_is_present = presence_of_speed;
      }
    else{
      throw std::invalid_argument("LocationContainer is not present!");
    }
  }

    /**
   * @brief Set the vehicle speed
   *
   * @param denm DENM to set the speed value
   * @param speed_val speed value to set in m/s as decimal number
   */
  inline void setSpeed(DENM& denm, const double speed_val){
    if(denm.denm.location_is_present){
      setSpeed(denm.denm.location.event_speed, speed_val);
      setIsSpeedPresent(denm, true);
      }
    else{
      throw std::invalid_argument("LocationContainer is not present!");
    }
  }

   /**
   * @brief Set the ReferencePosition of a DENM from a given UTM-Position
   * 
   * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
   * The z-Coordinate is directly used as altitude value
   * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
   * 
   * @param[out] denm DENM for which to set the ReferencePosition
   * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
   * @param[in] zone the UTM zone (zero means UPS) of the given position
   * @param[in] northp hemisphere (true means north, false means south)
   */
  inline void setFromUTMPosition(DENM& denm, const gm::PointStamped& utm_position, const int& zone, const bool& northp)
  {
    setFromUTMPosition(denm.denm.management.event_position, utm_position, zone, northp);
  }

} // namespace etsi_its_denm_msgs::access
