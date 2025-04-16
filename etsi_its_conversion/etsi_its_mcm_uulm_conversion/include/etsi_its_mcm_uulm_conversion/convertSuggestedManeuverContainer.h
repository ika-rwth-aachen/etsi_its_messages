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
  asn1/patched/uulm_mcm_etsi/TS103561_LUKAS_MCM.asn \
  asn1/patched/uulm_mcm_etsi/ETSI-ITS-CDD.asn \
  -t \
  mcm_uulm \
  -o \
  etsi_its_conversion/etsi_its_mcm_uulm_conversion/include/etsi_its_mcm_uulm_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
SuggestedManeuverContainer ::= SEQUENCE {
	targetStationID     StationId,
	suggestedManeuver   SuggestedManeuver OPTIONAL
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_mcm_uulm_coding/mcm_uulm_SuggestedManeuverContainer.h>
#include <etsi_its_mcm_uulm_conversion/convertStationId.h>
#include <etsi_its_mcm_uulm_conversion/convertSuggestedManeuver.h>
#ifdef ROS1
#include <etsi_its_mcm_uulm_msgs/SuggestedManeuverContainer.h>
namespace mcm_uulm_msgs = etsi_its_mcm_uulm_msgs;
#else
#include <etsi_its_mcm_uulm_msgs/msg/suggested_maneuver_container.hpp>
namespace mcm_uulm_msgs = etsi_its_mcm_uulm_msgs::msg;
#endif


namespace etsi_its_mcm_uulm_conversion {

void toRos_SuggestedManeuverContainer(const mcm_uulm_SuggestedManeuverContainer_t& in, mcm_uulm_msgs::SuggestedManeuverContainer& out) {
  toRos_StationId(in.targetStationID, out.target_station_id);
  if (in.suggestedManeuver) {
    toRos_SuggestedManeuver(*in.suggestedManeuver, out.suggested_maneuver);
    out.suggested_maneuver_is_present = true;
  }
}

void toStruct_SuggestedManeuverContainer(const mcm_uulm_msgs::SuggestedManeuverContainer& in, mcm_uulm_SuggestedManeuverContainer_t& out) {
  memset(&out, 0, sizeof(mcm_uulm_SuggestedManeuverContainer_t));
  toStruct_StationId(in.target_station_id, out.targetStationID);
  if (in.suggested_maneuver_is_present) {
    out.suggestedManeuver = (mcm_uulm_SuggestedManeuver_t*) calloc(1, sizeof(mcm_uulm_SuggestedManeuver_t));
    toStruct_SuggestedManeuver(in.suggested_maneuver, *out.suggestedManeuver);
  }
}

}
