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

#include <etsi_its_spatem_coding/RTCMheader.h>
#include <etsi_its_spatem_conversion/convertGNSSstatus.h>
#include <etsi_its_spatem_conversion/convertAntennaOffsetSet.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/RTCMheader.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/r_t_c_mheader.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_RTCMheader(const RTCMheader_t& in, spatem_msgs::RTCMheader& out) {

  toRos_GNSSstatus(in.status, out.status);
  toRos_AntennaOffsetSet(in.offsetSet, out.offset_set);
}

void toStruct_RTCMheader(const spatem_msgs::RTCMheader& in, RTCMheader_t& out) {

  memset(&out, 0, sizeof(RTCMheader_t));

  toStruct_GNSSstatus(in.status, out.status);
  toStruct_AntennaOffsetSet(in.offset_set, out.offsetSet);
}

}