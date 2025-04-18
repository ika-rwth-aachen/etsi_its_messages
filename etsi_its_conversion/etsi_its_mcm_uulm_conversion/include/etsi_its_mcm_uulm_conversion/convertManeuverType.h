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
ManeuverType ::= ENUMERATED {
	freeTravel           (0),
	followRoadUser       (1),
	crossIntersection    (2),
	yieldAtIntersection  (3),
	overtake             (4),
	pullOver             (5),
	yieldAtObstacle      (6),
	maneuverTypeUnknown  (15)
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_mcm_uulm_coding/mcm_uulm_ManeuverType.h>

#ifdef ROS1
#include <etsi_its_mcm_uulm_msgs/ManeuverType.h>
namespace mcm_uulm_msgs = etsi_its_mcm_uulm_msgs;
#else
#include <etsi_its_mcm_uulm_msgs/msg/maneuver_type.hpp>
namespace mcm_uulm_msgs = etsi_its_mcm_uulm_msgs::msg;
#endif


namespace etsi_its_mcm_uulm_conversion {

void toRos_ManeuverType(const mcm_uulm_ManeuverType_t& in, mcm_uulm_msgs::ManeuverType& out) {
  out.value = in;
}

void toStruct_ManeuverType(const mcm_uulm_msgs::ManeuverType& in, mcm_uulm_ManeuverType_t& out) {
  memset(&out, 0, sizeof(mcm_uulm_ManeuverType_t));
  out = in.value;
}

}
