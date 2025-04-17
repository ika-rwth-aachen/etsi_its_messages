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
LongitudinalManeuverWaypointContainer ::= SEQUENCE SIZE(1..128, ...) OF LongitudinalWaypoint
----------------------------------------------------------------------------- */

#pragma once

#include <stdexcept>

#include <etsi_its_mcm_uulm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mcm_uulm_coding/mcm_uulm_LongitudinalManeuverWaypointContainer.h>
#include <etsi_its_mcm_uulm_coding/mcm_uulm_LongitudinalWaypoint.h>
#include <etsi_its_mcm_uulm_conversion/convertLongitudinalWaypoint.h>
#ifdef ROS1
#include <etsi_its_mcm_uulm_msgs/LongitudinalWaypoint.h>
#include <etsi_its_mcm_uulm_msgs/LongitudinalManeuverWaypointContainer.h>
namespace mcm_uulm_msgs = etsi_its_mcm_uulm_msgs;
#else
#include <etsi_its_mcm_uulm_msgs/msg/longitudinal_waypoint.hpp>
#include <etsi_its_mcm_uulm_msgs/msg/longitudinal_maneuver_waypoint_container.hpp>
namespace mcm_uulm_msgs = etsi_its_mcm_uulm_msgs::msg;
#endif


namespace etsi_its_mcm_uulm_conversion {

void toRos_LongitudinalManeuverWaypointContainer(const mcm_uulm_LongitudinalManeuverWaypointContainer_t& in, mcm_uulm_msgs::LongitudinalManeuverWaypointContainer& out) {
  for (int i = 0; i < in.list.count; ++i) {
    mcm_uulm_msgs::LongitudinalWaypoint el;
    toRos_LongitudinalWaypoint(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_LongitudinalManeuverWaypointContainer(const mcm_uulm_msgs::LongitudinalManeuverWaypointContainer& in, mcm_uulm_LongitudinalManeuverWaypointContainer_t& out) {
  memset(&out, 0, sizeof(mcm_uulm_LongitudinalManeuverWaypointContainer_t));
  for (int i = 0; i < in.array.size(); ++i) {
    mcm_uulm_LongitudinalWaypoint_t* el = (mcm_uulm_LongitudinalWaypoint_t*) calloc(1, sizeof(mcm_uulm_LongitudinalWaypoint_t));
    toStruct_LongitudinalWaypoint(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
