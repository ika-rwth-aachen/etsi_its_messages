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
  asn1/raw/cam_ts103900/CAM-PDU-Descriptions.asn \
  asn1/patched/cam_ts103900/cdd/ETSI-ITS-CDD.asn \
  -t \
  cam_ts \
  -o \
  etsi_its_conversion/etsi_its_cam_ts_conversion/include/etsi_its_cam_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
*
* This type contains detaild information of the Emergency Container.
*
* It shall include the following components:
*
* @field lightBarSirenInUse: it indicates whether light-bar or a siren is in use by the vehicle originating the CAM.
*
* @field incidentIndication: the optional incident related to the roadworks to provide additional information of the roadworks zone.
*
* @field emergencyPriority: the optional component represent right of way indicator of the vehicle ITS-S that originates the CAM PDU.
*
EmergencyContainer ::= SEQUENCE {
	lightBarSirenInUse LightBarSirenInUse,
	incidentIndication CauseCodeV2 OPTIONAL,
	emergencyPriority EmergencyPriority OPTIONAL
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_cam_ts_coding/cam_ts_EmergencyContainer.h>
#include <etsi_its_cam_ts_conversion/convertCauseCodeV2.h>
#include <etsi_its_cam_ts_conversion/convertEmergencyPriority.h>
#include <etsi_its_cam_ts_conversion/convertLightBarSirenInUse.h>
#ifdef ROS1
#include <etsi_its_cam_ts_msgs/EmergencyContainer.h>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs;
#else
#include <etsi_its_cam_ts_msgs/msg/emergency_container.hpp>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs::msg;
#endif


namespace etsi_its_cam_ts_conversion {

void toRos_EmergencyContainer(const cam_ts_EmergencyContainer_t& in, cam_ts_msgs::EmergencyContainer& out) {
  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.light_bar_siren_in_use);
  if (in.incidentIndication) {
    toRos_CauseCodeV2(*in.incidentIndication, out.incident_indication);
    out.incident_indication_is_present = true;
  }
  if (in.emergencyPriority) {
    toRos_EmergencyPriority(*in.emergencyPriority, out.emergency_priority);
    out.emergency_priority_is_present = true;
  }
}

void toStruct_EmergencyContainer(const cam_ts_msgs::EmergencyContainer& in, cam_ts_EmergencyContainer_t& out) {
  memset(&out, 0, sizeof(cam_ts_EmergencyContainer_t));
  toStruct_LightBarSirenInUse(in.light_bar_siren_in_use, out.lightBarSirenInUse);
  if (in.incident_indication_is_present) {
    out.incidentIndication = (cam_ts_CauseCodeV2_t*) calloc(1, sizeof(cam_ts_CauseCodeV2_t));
    toStruct_CauseCodeV2(in.incident_indication, *out.incidentIndication);
  }
  if (in.emergency_priority_is_present) {
    out.emergencyPriority = (cam_ts_EmergencyPriority_t*) calloc(1, sizeof(cam_ts_EmergencyPriority_t));
    toStruct_EmergencyPriority(in.emergency_priority, *out.emergencyPriority);
  }
}

}
