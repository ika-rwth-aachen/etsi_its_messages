/** ============================================================================
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
============================================================================= */

/** Auto-generated by https://github.com/ika-rwth-aachen/etsi_its_messages -----
python3 \
  utils/codegen/codegen-py/asn1ToConversionHeader.py \
  asn1/patched/uulm_mcm_etsi/ETSI-ITS-CDD.asn \
  asn1/patched/uulm_mcm_etsi/TS103561_LUKAS_MCM.asn \
  -t \
  mcm_ts \
  -o \
  etsi_its_conversion/uulm_mcm_ts_conversion/include/uulm_mcm_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
YieldToRoadUserContainer ::= SEQUENCE SIZE(1..16, ...) OF ParticipatingRoadUserIndex
----------------------------------------------------------------------------- */

#pragma once

#include <stdexcept>

#include <etsi_its_mcm_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mcm_ts_coding/mcm_ts_YieldToRoadUserContainer.h>
#include <etsi_its_mcm_ts_coding/mcm_ts_ParticipatingRoadUserIndex.h>
#include <etsi_its_mcm_ts_conversion/convertParticipatingRoadUserIndex.h>
#ifdef ROS1
#include <etsi_its_mcm_ts_msgs/ParticipatingRoadUserIndex.h>
#include <etsi_its_mcm_ts_msgs/YieldToRoadUserContainer.h>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs;
#else
#include <etsi_its_mcm_ts_msgs/msg/participating_road_user_index.hpp>
#include <etsi_its_mcm_ts_msgs/msg/yield_to_road_user_container.hpp>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs::msg;
#endif


namespace etsi_its_mcm_ts_conversion {

void toRos_YieldToRoadUserContainer(const mcm_ts_YieldToRoadUserContainer_t& in, mcm_ts_msgs::YieldToRoadUserContainer& out) {
  for (int i = 0; i < in.list.count; ++i) {
    mcm_ts_msgs::ParticipatingRoadUserIndex el;
    toRos_ParticipatingRoadUserIndex(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_YieldToRoadUserContainer(const mcm_ts_msgs::YieldToRoadUserContainer& in, mcm_ts_YieldToRoadUserContainer_t& out) {
  memset(&out, 0, sizeof(mcm_ts_YieldToRoadUserContainer_t));
  for (int i = 0; i < in.array.size(); ++i) {
    mcm_ts_ParticipatingRoadUserIndex_t* el = (mcm_ts_ParticipatingRoadUserIndex_t*) calloc(1, sizeof(mcm_ts_ParticipatingRoadUserIndex_t));
    toStruct_ParticipatingRoadUserIndex(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
