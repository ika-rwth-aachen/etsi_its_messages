/*
=============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

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
 * @file impl/mapem/mapem_getters.h
 * @brief Getter functions for the ETSI ITS MAPEM
 */

#pragma once

namespace J2735 = etsi_its_msgs::J2735_access;

namespace etsi_its_mapem_msgs {

namespace access {

  /**
   * @brief Get the value of MinuteOfTheYear object from mapem
   * 
   * @param mapem object to get the MinuteOfTheYear
   * @return MinuteOfTheYear the minute of the year object
   */
  inline MinuteOfTheYear getMinuteOfTheYear(const MAPEM& mapem) {
    etsi_its_msgs::throwIfNotIsPresent(mapem.map.time_stamp_is_present, "mapem.map.time_stamp");
    return mapem.map.time_stamp;
  }

  /**
   * @brief Get the value of MinuteOfTheYear value from mapem
   * 
   * @param mapem object to get the MinuteOfTheYear value from
   * @return uint32_t the minute of the year value
   */
  inline uint32_t getMinuteOfTheYearValue(const MAPEM& mapem) {
    MinuteOfTheYear moy = getMinuteOfTheYear(mapem);
    return moy.value;
  }

} // namespace access

} // namespace etsi_its_mapem_msgs
