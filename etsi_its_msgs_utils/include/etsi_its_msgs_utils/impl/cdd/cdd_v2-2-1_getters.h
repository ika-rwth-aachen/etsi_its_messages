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
 * @file impl/cdd/cdd_v2-2-1_getters.h
 * @brief Getter functions for the ETSI ITS Common Data Dictionary (CDD) v2.2.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_2_1_GETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_2_1_GETTERS_H
#include <GeographicLib/UTMUPS.hpp>

#include <etsi_its_msgs_utils/impl/cdd/cdd_getters_common.h>

/**
 * @brief Get the WGS Heading value
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading to get the WGS Heading value from
 * @return WGS Heading value in degree as decimal number
 */
template <typename Wgs84Angle>
inline double getWGSHeadingCDD(const Wgs84Angle& heading) { return ((double)heading.value.value) * 1e-1; }

/**
 * @brief Get the WGS Heading confidence
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading to get the WGS Heading standard deviation from
 * @return WGS Heading standard deviation in degree as decimal number
 */

template <typename Wgs84Angle>
inline double getWGSHeadingConfidenceCDD(const Wgs84Angle& heading) { return ((double)heading.confidence.value) * 1e-1 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR; }

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_2_1_GETTERS_H