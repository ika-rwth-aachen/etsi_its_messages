// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/mcm/mcm_access.h
 * @brief Main MCM access implementation header
 */

#undef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_MCM_MCM_UTILS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CHECKS_H

#pragma once

#include <cstring>
#include <iostream>
#include <map>

#include <GeographicLib/UTMUPS.hpp>

#include <etsi_its_msgs_utils/impl/constants.h>
#include <etsi_its_msgs_utils/impl/mcm/mcm_getters.h>
#include <etsi_its_msgs_utils/impl/mcm/mcm_setters.h>

namespace etsi_its_mcm_uulm_msgs::access {
#include <etsi_its_msgs_utils/impl/mcm/mcm_utils.h>
}  // namespace etsi_its_mcm_uulm_msgs::access