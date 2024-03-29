/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include <etsi_its_cam_coding/VehicleIdentification.h>
#include <etsi_its_cam_conversion/convertWMInumber.h>
#include <etsi_its_cam_conversion/convertVDS.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VehicleIdentification.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vehicle_identification.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VehicleIdentification(const VehicleIdentification_t& in, cam_msgs::VehicleIdentification& out) {

  if (in.wMInumber) {
    toRos_WMInumber(*in.wMInumber, out.wm_inumber);
    out.wm_inumber_is_present = true;
  }

  if (in.vDS) {
    toRos_VDS(*in.vDS, out.vds);
    out.vds_is_present = true;
  }

}

void toStruct_VehicleIdentification(const cam_msgs::VehicleIdentification& in, VehicleIdentification_t& out) {

  memset(&out, 0, sizeof(VehicleIdentification_t));

  if (in.wm_inumber_is_present) {
    WMInumber_t wm_inumber;
    toStruct_WMInumber(in.wm_inumber, wm_inumber);
    out.wMInumber = new WMInumber_t(wm_inumber);
  }

  if (in.vds_is_present) {
    VDS_t vds;
    toStruct_VDS(in.vds, vds);
    out.vDS = new VDS_t(vds);
  }

}

}