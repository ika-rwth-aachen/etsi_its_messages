// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

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