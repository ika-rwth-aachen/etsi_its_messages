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

#include <etsi_its_spatem_coding/SignalStatus.h>
#include <etsi_its_spatem_conversion/convertMsgCount.h>
#include <etsi_its_spatem_conversion/convertIntersectionReferenceID.h>
#include <etsi_its_spatem_conversion/convertSignalStatusPackageList.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/SignalStatus.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/signal_status.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_SignalStatus(const SignalStatus_t& in, spatem_msgs::SignalStatus& out) {

  toRos_MsgCount(in.sequenceNumber, out.sequence_number);
  toRos_IntersectionReferenceID(in.id, out.id);
  toRos_SignalStatusPackageList(in.sigStatus, out.sig_status);
  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_SignalStatus(const spatem_msgs::SignalStatus& in, SignalStatus_t& out) {

  memset(&out, 0, sizeof(SignalStatus_t));

  toStruct_MsgCount(in.sequence_number, out.sequenceNumber);
  toStruct_IntersectionReferenceID(in.id, out.id);
  toStruct_SignalStatusPackageList(in.sig_status, out.sigStatus);
  if (in.regional_is_present) {
    RegionalExtension_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_t(regional);
  }

}

}