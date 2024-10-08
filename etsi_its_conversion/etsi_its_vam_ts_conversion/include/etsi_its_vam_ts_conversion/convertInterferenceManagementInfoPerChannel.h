/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro

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

// --- Auto-generated by asn1ToConversionHeader.py -----------------------------

#pragma once

#include <etsi_its_vam_ts_coding/vam_ts_InterferenceManagementInfoPerChannel.h>
#include <etsi_its_vam_ts_conversion/convertInterferenceManagementChannel.h>
#include <etsi_its_vam_ts_conversion/convertInterferenceManagementZoneType.h>
#include <etsi_its_vam_ts_conversion/convertMitigationForTechnologies.h>
#include <etsi_its_vam_ts_conversion/convertTimestampIts.h>
#ifdef ROS1
#include <etsi_its_vam_ts_msgs/InterferenceManagementInfoPerChannel.h>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs;
#else
#include <etsi_its_vam_ts_msgs/msg/interference_management_info_per_channel.hpp>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs::msg;
#endif


namespace etsi_its_vam_ts_conversion {

void toRos_InterferenceManagementInfoPerChannel(const vam_ts_InterferenceManagementInfoPerChannel_t& in, vam_ts_msgs::InterferenceManagementInfoPerChannel& out) {
  toRos_InterferenceManagementChannel(in.interferenceManagementChannel, out.interference_management_channel);
  toRos_InterferenceManagementZoneType(in.interferenceManagementZoneType, out.interference_management_zone_type);
  if (in.interferenceManagementMitigationType) {
    toRos_MitigationForTechnologies(*in.interferenceManagementMitigationType, out.interference_management_mitigation_type);
    out.interference_management_mitigation_type_is_present = true;
  }
  if (in.expiryTime) {
    toRos_TimestampIts(*in.expiryTime, out.expiry_time);
    out.expiry_time_is_present = true;
  }
}

void toStruct_InterferenceManagementInfoPerChannel(const vam_ts_msgs::InterferenceManagementInfoPerChannel& in, vam_ts_InterferenceManagementInfoPerChannel_t& out) {
  memset(&out, 0, sizeof(vam_ts_InterferenceManagementInfoPerChannel_t));

  toStruct_InterferenceManagementChannel(in.interference_management_channel, out.interferenceManagementChannel);
  toStruct_InterferenceManagementZoneType(in.interference_management_zone_type, out.interferenceManagementZoneType);
  if (in.interference_management_mitigation_type_is_present) {
    out.interferenceManagementMitigationType = (vam_ts_MitigationForTechnologies_t*) calloc(1, sizeof(vam_ts_MitigationForTechnologies_t));
    toStruct_MitigationForTechnologies(in.interference_management_mitigation_type, *out.interferenceManagementMitigationType);
  }
  if (in.expiry_time_is_present) {
    out.expiryTime = (vam_ts_TimestampIts_t*) calloc(1, sizeof(vam_ts_TimestampIts_t));
    toStruct_TimestampIts(in.expiry_time, *out.expiryTime);
  }
}

}
