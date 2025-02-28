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
AdviceResponse ::= SEQUENCE {
	maneuverID      ManeuverID,
	adviceUpdateID  AdviceUpdateID,
	adviceFollowed  ManeuverResponse
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_mcm_ts_coding/mcm_ts_AdviceResponse.h>
#include <etsi_its_mcm_ts_conversion/convertAdviceUpdateID.h>
#include <etsi_its_mcm_ts_conversion/convertManeuverID.h>
#include <etsi_its_mcm_ts_conversion/convertManeuverResponse.h>
#ifdef ROS1
#include <etsi_its_mcm_ts_msgs/AdviceResponse.h>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs;
#else
#include <etsi_its_mcm_ts_msgs/msg/advice_response.hpp>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs::msg;
#endif


namespace etsi_its_mcm_ts_conversion {

void toRos_AdviceResponse(const mcm_ts_AdviceResponse_t& in, mcm_ts_msgs::AdviceResponse& out) {
  toRos_ManeuverID(in.maneuverID, out.maneuver_id);
  toRos_AdviceUpdateID(in.adviceUpdateID, out.advice_update_id);
  toRos_ManeuverResponse(in.adviceFollowed, out.advice_followed);
}

void toStruct_AdviceResponse(const mcm_ts_msgs::AdviceResponse& in, mcm_ts_AdviceResponse_t& out) {
  memset(&out, 0, sizeof(mcm_ts_AdviceResponse_t));
  toStruct_ManeuverID(in.maneuver_id, out.maneuverID);
  toStruct_AdviceUpdateID(in.advice_update_id, out.adviceUpdateID);
  toStruct_ManeuverResponse(in.advice_followed, out.adviceFollowed);
}

}
