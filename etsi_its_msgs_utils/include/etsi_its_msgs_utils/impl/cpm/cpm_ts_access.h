// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/cpm/cpm_access.h
 * @brief Main CPM access implementation header
 */

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
#include <etsi_its_msgs_utils/impl/cpm/cpm_ts_getters.h>
#include <etsi_its_msgs_utils/impl/cpm/cpm_ts_setters.h>

namespace etsi_its_cpm_ts_msgs::access {
#include <etsi_its_msgs_utils/impl/cpm/cpm_ts_utils.h>
}  // namespace etsi_its_cpm_ts_msgs::access