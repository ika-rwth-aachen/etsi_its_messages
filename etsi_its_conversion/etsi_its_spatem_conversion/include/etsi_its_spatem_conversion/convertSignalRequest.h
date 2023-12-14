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

#include <etsi_its_spatem_coding/SignalRequest.h>
#include <etsi_its_spatem_conversion/convertIntersectionReferenceID.h>
#include <etsi_its_spatem_conversion/convertRequestID.h>
#include <etsi_its_spatem_conversion/convertPriorityRequestType.h>
#include <etsi_its_spatem_conversion/convertIntersectionAccessPoint.h>
#include <etsi_its_spatem_conversion/convertIntersectionAccessPoint.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/SignalRequest.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/signal_request.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_SignalRequest(const SignalRequest_t& in, spatem_msgs::SignalRequest& out) {

  toRos_IntersectionReferenceID(in.id, out.id);
  toRos_RequestID(in.requestID, out.request_id);
  toRos_PriorityRequestType(in.requestType, out.request_type);
  toRos_IntersectionAccessPoint(in.inBoundLane, out.in_bound_lane);
  if (in.outBoundLane) {
    toRos_IntersectionAccessPoint(*in.outBoundLane, out.out_bound_lane);
    out.out_bound_lane_is_present = true;
  }

  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_SignalRequest(const spatem_msgs::SignalRequest& in, SignalRequest_t& out) {

  memset(&out, 0, sizeof(SignalRequest_t));

  toStruct_IntersectionReferenceID(in.id, out.id);
  toStruct_RequestID(in.request_id, out.requestID);
  toStruct_PriorityRequestType(in.request_type, out.requestType);
  toStruct_IntersectionAccessPoint(in.in_bound_lane, out.inBoundLane);
  if (in.out_bound_lane_is_present) {
    IntersectionAccessPoint_t out_bound_lane;
    toStruct_IntersectionAccessPoint(in.out_bound_lane, out_bound_lane);
    out.outBoundLane = new IntersectionAccessPoint_t(out_bound_lane);
  }

  if (in.regional_is_present) {
    RegionalExtension_364P0_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_364P0_t(regional);
  }

}

}