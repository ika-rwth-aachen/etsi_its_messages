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
RoadUserState ::= SEQUENCE {
	roadUserType    RoadUserType,
	speed           SpeedValue,
	heading         HeadingValue,
	length          RoadUserLength,
	width           RoadUserWidth
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_mcm_ts_coding/mcm_ts_RoadUserState.h>
#include <etsi_its_mcm_ts_conversion/convertHeadingValue.h>
#include <etsi_its_mcm_ts_conversion/convertRoadUserLength.h>
#include <etsi_its_mcm_ts_conversion/convertRoadUserType.h>
#include <etsi_its_mcm_ts_conversion/convertRoadUserWidth.h>
#include <etsi_its_mcm_ts_conversion/convertSpeedValue.h>
#ifdef ROS1
#include <etsi_its_mcm_ts_msgs/RoadUserState.h>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs;
#else
#include <etsi_its_mcm_ts_msgs/msg/road_user_state.hpp>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs::msg;
#endif


namespace etsi_its_mcm_ts_conversion {

void toRos_RoadUserState(const mcm_ts_RoadUserState_t& in, mcm_ts_msgs::RoadUserState& out) {
  toRos_RoadUserType(in.roadUserType, out.road_user_type);
  toRos_SpeedValue(in.speed, out.speed);
  toRos_HeadingValue(in.heading, out.heading);
  toRos_RoadUserLength(in.length, out.length);
  toRos_RoadUserWidth(in.width, out.width);
}

void toStruct_RoadUserState(const mcm_ts_msgs::RoadUserState& in, mcm_ts_RoadUserState_t& out) {
  memset(&out, 0, sizeof(mcm_ts_RoadUserState_t));
  toStruct_RoadUserType(in.road_user_type, out.roadUserType);
  toStruct_SpeedValue(in.speed, out.speed);
  toStruct_HeadingValue(in.heading, out.heading);
  toStruct_RoadUserLength(in.length, out.length);
  toStruct_RoadUserWidth(in.width, out.width);
}

}
