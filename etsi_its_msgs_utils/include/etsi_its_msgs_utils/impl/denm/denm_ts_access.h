// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/denm/denm_ts_access.h
 * @brief Main DENM TS access implementation header
 */

#undef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_DENM_DENM_GETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_DENM_DENM_SETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_DENM_DENM_UTILS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CHECKS_H

#pragma once

#include <cstring>
#include <iostream>
#include <map>

#include <GeographicLib/UTMUPS.hpp>

#include <etsi_its_msgs_utils/impl/constants.h>
#include <etsi_its_msgs_utils/impl/denm/denm_ts_getters.h>
#include <etsi_its_msgs_utils/impl/denm/denm_ts_setters.h>

namespace etsi_its_denm_ts_msgs::access {
#include <etsi_its_msgs_utils/impl/denm/denm_utils.h>
}  // namespace etsi_its_denm_ts_msgs::access