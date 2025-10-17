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
 * @file impl/mapem/mapem_ts_getters.h
 * @brief Getter functions for the ETSI ITS MAPEM
 */

#pragma once

namespace etsi_its_mapem_ts_msgs {

namespace access {

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_getters.h>
#include <etsi_its_msgs_utils/impl/checks.h>

    /**
   * @brief Get the value of MinuteOfTheYear object MapData object
   * 
   * @param map object to get the MinuteOfTheYear from
   * @return MinuteOfTheYear the minute of the year object
   */
  inline MinuteOfTheYear getMinuteOfTheYear(const MapData& map) {
    throwIfNotPresent(map.time_stamp_is_present, "map.time_stamp");
    return map.time_stamp;
  }

  /**
   * @brief Get the value of MinuteOfTheYear value from MapData object
   * 
   * @param map object to get the MinuteOfTheYear value from
   * @return uint32_t the minute of the year value
   */
  inline uint32_t getMinuteOfTheYearValue(const MapData& map) {
    MinuteOfTheYear moy = getMinuteOfTheYear(map);
    return moy.value;
  }

  /**
   * @brief Get the value of MinuteOfTheYear object from mapem
   * 
   * @param mapem object to get the MinuteOfTheYear
   * @return MinuteOfTheYear the minute of the year object
   */
  inline MinuteOfTheYear getMinuteOfTheYear(const MAPEM& mapem) {
    return getMinuteOfTheYear(mapem.map);
  }

  /**
   * @brief Get the value of MinuteOfTheYear value from mapem
   * 
   * @param mapem object to get the MinuteOfTheYear value from
   * @return uint32_t the minute of the year value
   */
  inline uint32_t getMinuteOfTheYearValue(const MAPEM& mapem) {
    return getMinuteOfTheYearValue(mapem.map);
  }

  /**
   * @brief Get the IntersectionID value
   * 
   * @param intsct_id IntersectionID object to get the value from
   * @return uint16_t the IntersectionID value
   */
  inline uint16_t getIntersectionID(const IntersectionID& intsct_id) {
    return intsct_id.value;
  }

  /**
   * @brief Get the IntersectionId of an IntersectionGeometry object
   * 
   * @param intsct IntersectionGeometry object
   * @return uint16_t the IntersectionId value
   */
  inline uint16_t getIntersectionID(const IntersectionGeometry& intsct) {
    return getIntersectionID(intsct.id.id);
  }

  /**
   * @brief Get the Latitude value
   *
   * @param latitude object to get the Latitude value from
   * @return Latitude value in degree as decimal number
   */
  inline double getLatitude(const Latitude& latitude) {
    return ((double)latitude.value)*1e-7;
  }

  /**
   * @brief Get the Longitude value
   *
   * @param longitude object to get the Longitude value from
   * @return Longitude value in degree as decimal number
   */
  inline double getLongitude(const Longitude& longitude) {
    return ((double)longitude.value)*1e-7;
  }

  /**
   * @brief Get the Elevation value
   *
   * @param elevation object to get the Elevation value from
   * @return Elevation value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline double getElevation(const Elevation& elevation) {
    return ((double)elevation.value)*1e-1;
  }

  /**
   * @brief Get the Latitude value from a given Position3D object
   *
   * @param ref_point Position3D object to get the Latitude value from
   * @return Latitude value in degree as decimal number
   */
  inline double getLatitude(const Position3D& ref_point) {
    return getLatitude(ref_point.lat);
  }

  /**
   * @brief Get the Longitude value from a given Position3D object
   *
   * @param ref_point Position3D object to get the Longitude value from
   * @return Longitude value in degree as decimal number
   */
  inline double getLongitude(const Position3D& ref_point) {
    return getLongitude(ref_point.lon);
  }

  /**
   * @brief Get the Elevation value from a given Position3D object
   *
   * @param ref_point Position3D object to get the Elevation value from
   * @return Elevation value (above the reference ellipsoid surface) in meter as decimal number
   */
  inline double getElevation(const Position3D& ref_point) {
    throwIfNotPresent(ref_point.elevation_is_present, "Position3D.elevation_is_present");
    return getElevation(ref_point.elevation);
  }

    /**
   * @brief Get the LaneDirection in form of bool vector
   *
   * @param lane_direction LaneDirection object to get the bool vector from
   * @return std::vector<bool>
   */
  inline std::vector<bool> getLaneDirection(const LaneDirection& lane_direction) {
    return getBitString(lane_direction.value, lane_direction.bits_unused);
  }

  /**
   * @brief Get the LaneDirection in form of bool vector from a LaneAttributes object
   *
   * @param lane_attributes LaneAttributes object to get the bool vector from
   * @return std::vector<bool>
   */
  inline std::vector<bool> getLaneDirection(const LaneAttributes& lane_attributes) {
    return getLaneDirection(lane_attributes.directional_use);
  }

  /**
   * @brief Get the LaneDirection in form of bool vector from a GenericLane object
   *
   * @param generic_lane GenericLane object to get the bool vector from
   * @return std::vector<bool>
   */
  inline std::vector<bool> getLaneDirection(const GenericLane& generic_lane) {
    return getLaneDirection(generic_lane.lane_attributes);
  }

  /**
   * @brief Get the Point From NodeXY object
   * 
   * @tparam T representing different NodeXY types (NodeXY20b, NodeXY22b...)
   * @param node_xy the NodeXY object
   * @return gm::Point geometry_msgs::Point representing the node point (values given in meters)
   */
  template <typename T>
  inline gm::Point getPointFromNodeXY(const T& node_xy) {
    gm::Point p;
    p.x = ((double)node_xy.x.value) * 1e-2;
    p.y = ((double)node_xy.y.value) * 1e-2;
    return p;
  }

} // namespace access

} // namespace etsi_its_mapem_ts_msgs
