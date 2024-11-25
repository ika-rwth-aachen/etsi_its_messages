/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include <etsi_its_denm_coding/denm_DENM.h>
#include <etsi_its_denm_conversion/convertDecentralizedEnvironmentalNotificationMessage.h>
#include <etsi_its_denm_conversion/convertItsPduHeader.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/DENM.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/denm.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DENM(const denm_DENM_t& in, denm_msgs::DENM& out) {
  toRos_ItsPduHeader(in.header, out.header);
  toRos_DecentralizedEnvironmentalNotificationMessage(in.denm, out.denm);
}

void toStruct_DENM(const denm_msgs::DENM& in, denm_DENM_t& out) {
  memset(&out, 0, sizeof(denm_DENM_t));

  toStruct_ItsPduHeader(in.header, out.header);
  toStruct_DecentralizedEnvironmentalNotificationMessage(in.denm, out.denm);
}

}
