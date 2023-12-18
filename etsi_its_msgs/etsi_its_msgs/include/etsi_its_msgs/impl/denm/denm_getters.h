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
 * @file impl/denm/denm_getters.h
 * @brief Getter functions for the ETSI ITS DENM
 */

#pragma once

namespace cdd = etsi_its_msgs::cdd_access;
namespace etsi_its_denm_msgs {

namespace access {


  /**
   * @brief Get the Station ID object
   *
   * @param denm DENM to get the StationID value from
   * @return stationID value
   */
  inline uint32_t getStationID(const DENM& denm){
    return cdd::getStationID(denm.header);
  }

  /**
   * @brief Get the stationType object
   * 
   * @param denm DENM to get the stationType value from
   * @return stationType value
   */
  inline uint8_t getStationType(const DENM& denm){
    return denm.denm.management.station_type.value;
  }

} // namespace access

} // namespace etsi_its_denm_msgs
