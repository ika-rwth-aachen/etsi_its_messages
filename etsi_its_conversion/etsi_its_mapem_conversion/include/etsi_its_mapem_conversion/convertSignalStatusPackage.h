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

#include <etsi_its_mapem_coding/SignalStatusPackage.h>
#include <etsi_its_mapem_conversion/convertSignalRequesterInfo.h>
#include <etsi_its_mapem_conversion/convertIntersectionAccessPoint.h>
#include <etsi_its_mapem_conversion/convertIntersectionAccessPoint.h>
#include <etsi_its_mapem_conversion/convertMinuteOfTheYear.h>
#include <etsi_its_mapem_conversion/convertDSecond.h>
#include <etsi_its_mapem_conversion/convertDSecond.h>
#include <etsi_its_mapem_conversion/convertPrioritizationResponseStatus.h>
#include <etsi_its_mapem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/SignalStatusPackage.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/signal_status_package.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_SignalStatusPackage(const SignalStatusPackage_t& in, mapem_msgs::SignalStatusPackage& out) {

  if (in.requester) {
    toRos_SignalRequesterInfo(*in.requester, out.requester);
    out.requester_is_present = true;
  }

  toRos_IntersectionAccessPoint(in.inboundOn, out.inbound_on);
  if (in.outboundOn) {
    toRos_IntersectionAccessPoint(*in.outboundOn, out.outbound_on);
    out.outbound_on_is_present = true;
  }

  if (in.minute) {
    toRos_MinuteOfTheYear(*in.minute, out.minute);
    out.minute_is_present = true;
  }

  if (in.second) {
    toRos_DSecond(*in.second, out.second);
    out.second_is_present = true;
  }

  if (in.duration) {
    toRos_DSecond(*in.duration, out.duration);
    out.duration_is_present = true;
  }

  toRos_PrioritizationResponseStatus(in.status, out.status);
  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_SignalStatusPackage(const mapem_msgs::SignalStatusPackage& in, SignalStatusPackage_t& out) {

  memset(&out, 0, sizeof(SignalStatusPackage_t));

  if (in.requester_is_present) {
    SignalRequesterInfo_t requester;
    toStruct_SignalRequesterInfo(in.requester, requester);
    out.requester = new SignalRequesterInfo_t(requester);
  }

  toStruct_IntersectionAccessPoint(in.inbound_on, out.inboundOn);
  if (in.outbound_on_is_present) {
    IntersectionAccessPoint_t outbound_on;
    toStruct_IntersectionAccessPoint(in.outbound_on, outbound_on);
    out.outboundOn = new IntersectionAccessPoint_t(outbound_on);
  }

  if (in.minute_is_present) {
    MinuteOfTheYear_t minute;
    toStruct_MinuteOfTheYear(in.minute, minute);
    out.minute = new MinuteOfTheYear_t(minute);
  }

  if (in.second_is_present) {
    DSecond_t second;
    toStruct_DSecond(in.second, second);
    out.second = new DSecond_t(second);
  }

  if (in.duration_is_present) {
    DSecond_t duration;
    toStruct_DSecond(in.duration, duration);
    out.duration = new DSecond_t(duration);
  }

  toStruct_PrioritizationResponseStatus(in.status, out.status);
  if (in.regional_is_present) {
    RegionalExtension_364P0_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_364P0_t(regional);
  }

}

}