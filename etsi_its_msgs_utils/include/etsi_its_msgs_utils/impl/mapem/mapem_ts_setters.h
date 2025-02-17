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
 * @file impl/mapem/mapem_ts_setters.h
 * @brief Setter functions for the ETSI ITS MAPEM
 */

#pragma once


#include <GeographicLib/UTMUPS.hpp>

namespace etsi_its_mapem_ts_msgs {

namespace access {

#include <etsi_its_msgs_utils/impl/checks.h>

  /**
   * @brief Set the MinuteOfTheYear object
   * 
   * @param moy MinuteOfTheYear object
   * @param moy_value value to set
   */
  inline void setMinuteOfTheYear(MinuteOfTheYear& moy, const uint32_t moy_value) {
    throwIfOutOfRange(moy_value, MinuteOfTheYear::MIN, MinuteOfTheYear::MAX, "MinuteOfTheYear");
    moy.value = moy_value;
  }

  /**
   * @brief Set the MinuteOfTheYear for a given MapData object
   * 
   * @param map MapData object
   * @param moy_value value to set
   */
  inline void setMinuteOfTheYear(MapData& map, const uint32_t moy_value) {
    setMinuteOfTheYear(map.time_stamp, moy_value);
  }

  /**
   * @brief Set the Minute Of The Year object
   * 
   * @param mapem 
   */
  inline void setMinuteOfTheYear(MAPEM& mapem, const uint32_t moy_value) {
    setMinuteOfTheYear(mapem.map, moy_value);
    mapem.map.time_stamp_is_present = true;
  }

  /**
   * @brief Set the IntersectionID value
   * 
   * @param intsct_id IntersectionID object
   * @param id_value value to set
   */
  inline void setIntersectionID(IntersectionID& intsct_id, const uint16_t id_value) {
    throwIfOutOfRange(id_value, IntersectionID::MIN, IntersectionID::MAX, "IntersectionID");
    intsct_id.value = id_value;
  }

  /**
   * @brief Set the IntersectionID for an IntersectionGeometry object
   * 
   * @param intsct IntersectionGeometry object
   * @param id_value value to set
   */
  inline void setIntersectionID(IntersectionGeometry& intsct, const uint16_t id_value) {
    setIntersectionID(intsct.id.id, id_value);
  }

  /**
   * @brief Set the Latitude object
   *
   * @param latitude object to set
   * @param deg Latitude value in degree as decimal number
   */
  inline void setLatitude(Latitude& latitude, const double deg) {
    int64_t angle_in_10_micro_degree = (int64_t)std::round(deg*1e7);
    throwIfOutOfRange(angle_in_10_micro_degree, Latitude::MIN, Latitude::MAX, "Latitude");
    latitude.value = angle_in_10_micro_degree;
  }

  /**
   * @brief Set the Longitude object
   *
   * @param longitude object to set
   * @param deg Longitude value in degree as decimal number
   */
  inline void setLongitude(Longitude& longitude, const double deg) {
    int64_t angle_in_10_micro_degree = (int64_t)std::round(deg*1e7);
    throwIfOutOfRange(angle_in_10_micro_degree, Longitude::MIN, Longitude::MAX, "Longitude");
    longitude.value = angle_in_10_micro_degree;
  }

  /**
   * @brief Set the Elevation object
   *
   * @param elevation object to set
   * @param value Elevation value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setElevation(Elevation& elevation, const double value) {
    int64_t alt_in_dm = (int64_t)std::round(value*1e1);
    if(alt_in_dm>=Elevation::MIN && alt_in_dm<=Elevation::MAX) elevation.value = alt_in_dm;
    else if(alt_in_dm<Elevation::MIN) elevation.value = Elevation::MIN;
    else if(alt_in_dm>Elevation::MAX) elevation.value = Elevation::MAX;
  }

  /**
   * @brief Set the Position3D object
   *
   * @param pos object to set
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   */
  inline void setPosition3D(Position3D& pos, const double latitude, const double longitude) {
    setLatitude(pos.lat, latitude);
    setLongitude(pos.lon, longitude);
    pos.elevation_is_present = false;
  }

  /**
   * @brief Set the Position3D object
   *
   * @param pos object to set
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   * @param altitude Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setPosition3D(Position3D& pos, const double latitude, const double longitude, const double altitude) {
    setPosition3D(pos, latitude, longitude);
    setElevation(pos.elevation, altitude);
    pos.elevation_is_present = true;
  }

    /**
   * @brief Set the Position3D of IntersectionGeometry object
   *
   * @param intsct IntersectionGeometry object
   * @param latitude Latitude value in degree as decimal number
   * @param longitude Longitude value in degree as decimal number
   * @param altitude Altitude value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline void setPosition3D(IntersectionGeometry& intsct, double latitude, double longitude, double altitude) {
    setPosition3D(intsct.ref_point, latitude, longitude, altitude);
  }

  /**
   * @brief Set the Position3D from a given UTM-Position
   *
   * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
   * The z-Coordinate is directly used as altitude value
   * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
   *
   * @param[out] reference_position Position3D to set
   * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
   * @param[in] zone the UTM zone (zero means UPS) of the given position
   * @param[in] northp hemisphere (true means north, false means south)
   */
  inline void setPosition3DFromUTMPosition(Position3D& reference_position, const gm::PointStamped& utm_position, const int zone, const bool northp) {
    std::string required_frame_prefix = "utm_";
    if(utm_position.header.frame_id.rfind(required_frame_prefix, 0) != 0)
    {
      throw std::invalid_argument("Frame-ID of UTM Position '"+utm_position.header.frame_id+"' does not start with required prefix '"+required_frame_prefix+"'!");
    }
    double latitude, longitude;
    try {
      GeographicLib::UTMUPS::Reverse(zone, northp, utm_position.point.x, utm_position.point.y, latitude, longitude);
    } catch (GeographicLib::GeographicErr& e) {
      throw std::invalid_argument(e.what());
    }
    setPosition3D(reference_position, latitude, longitude, utm_position.point.z);
  }

} // namespace access

} // namespace etsi_its_mapem_ts_msgs
