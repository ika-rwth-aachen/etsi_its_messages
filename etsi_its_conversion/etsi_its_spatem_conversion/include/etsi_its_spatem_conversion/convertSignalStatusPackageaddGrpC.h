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

#include <etsi_its_spatem_coding/SignalStatusPackage-addGrpC.h>
#include <etsi_its_spatem_conversion/convertDeltaTime.h>
#include <etsi_its_spatem_conversion/convertRejectedReason.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/SignalStatusPackage-addGrpC.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/signal_status_package_add_grp_c.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_SignalStatusPackage-addGrpC(const SignalStatusPackage-addGrpC_t& in, spatem_msgs::SignalStatusPackage-addGrpC& out) {

  if (in.synchToSchedule) {
    toRos_DeltaTime(*in.synchToSchedule, out.synch_to_schedule);
    out.synch_to_schedule_is_present = true;
  }

  if (in.rejectedReason) {
    toRos_RejectedReason(*in.rejectedReason, out.rejected_reason);
    out.rejected_reason_is_present = true;
  }

}

void toStruct_SignalStatusPackage-addGrpC(const spatem_msgs::SignalStatusPackage-addGrpC& in, SignalStatusPackage-addGrpC_t& out) {

  memset(&out, 0, sizeof(SignalStatusPackage-addGrpC_t));

  if (in.synch_to_schedule_is_present) {
    DeltaTime_t synch_to_schedule;
    toStruct_DeltaTime(in.synch_to_schedule, synch_to_schedule);
    out.synchToSchedule = new DeltaTime_t(synch_to_schedule);
  }

  if (in.rejected_reason_is_present) {
    RejectedReason_t rejected_reason;
    toStruct_RejectedReason(in.rejected_reason, rejected_reason);
    out.rejectedReason = new RejectedReason_t(rejected_reason);
  }

}

}