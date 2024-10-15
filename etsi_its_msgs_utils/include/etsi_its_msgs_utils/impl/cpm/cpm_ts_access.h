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
 * @file impl/cpm/cpm_access.h
 * @brief Main CPM access implementation header
 */

#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_CHECKS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V1_3_1_SETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_GETTERS_H
#undef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_V2_1_1_SETTERS_H

#pragma once

#include <cstring>
#include <iostream>
#include <map>

#include <GeographicLib/UTMUPS.hpp>

#include <etsi_its_msgs_utils/impl/cpm/cpm_ts_getters.h>
#include <etsi_its_msgs_utils/impl/cpm/cpm_ts_setters.h>

namespace etsi_its_cpm_ts_msgs::access {
#include <etsi_its_msgs_utils/impl/cpm/cpm_ts_utils.h>
}  // namespace etsi_its_cpm_ts_msgs::access