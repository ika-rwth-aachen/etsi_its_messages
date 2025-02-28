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
ManeuverConstraints ::= SEQUENCE {
	maneuverType                            ManeuverType OPTIONAL,
	longitudinalManeuverWaypointContainer   LongitudinalManeuverWaypointContainer,
	maneuverCorridor                        Polygon OPTIONAL
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_mcm_ts_coding/mcm_ts_ManeuverConstraints.h>
#include <etsi_its_mcm_ts_conversion/convertLongitudinalManeuverWaypointContainer.h>
#include <etsi_its_mcm_ts_conversion/convertManeuverType.h>
#include <etsi_its_mcm_ts_conversion/convertPolygon.h>
#ifdef ROS1
#include <etsi_its_mcm_ts_msgs/ManeuverConstraints.h>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs;
#else
#include <etsi_its_mcm_ts_msgs/msg/maneuver_constraints.hpp>
namespace mcm_ts_msgs = etsi_its_mcm_ts_msgs::msg;
#endif


namespace etsi_its_mcm_ts_conversion {

void toRos_ManeuverConstraints(const mcm_ts_ManeuverConstraints_t& in, mcm_ts_msgs::ManeuverConstraints& out) {
  if (in.maneuverType) {
    toRos_ManeuverType(*in.maneuverType, out.maneuver_type);
    out.maneuver_type_is_present = true;
  }
  toRos_LongitudinalManeuverWaypointContainer(in.longitudinalManeuverWaypointContainer, out.longitudinal_maneuver_waypoint_container);
  if (in.maneuverCorridor) {
    toRos_Polygon(*in.maneuverCorridor, out.maneuver_corridor);
    out.maneuver_corridor_is_present = true;
  }
}

void toStruct_ManeuverConstraints(const mcm_ts_msgs::ManeuverConstraints& in, mcm_ts_ManeuverConstraints_t& out) {
  memset(&out, 0, sizeof(mcm_ts_ManeuverConstraints_t));
  if (in.maneuver_type_is_present) {
    out.maneuverType = (mcm_ts_ManeuverType_t*) calloc(1, sizeof(mcm_ts_ManeuverType_t));
    toStruct_ManeuverType(in.maneuver_type, *out.maneuverType);
  }
  toStruct_LongitudinalManeuverWaypointContainer(in.longitudinal_maneuver_waypoint_container, out.longitudinalManeuverWaypointContainer);
  if (in.maneuver_corridor_is_present) {
    out.maneuverCorridor = (mcm_ts_Polygon_t*) calloc(1, sizeof(mcm_ts_Polygon_t));
    toStruct_Polygon(in.maneuver_corridor, *out.maneuverCorridor);
  }
}

}
