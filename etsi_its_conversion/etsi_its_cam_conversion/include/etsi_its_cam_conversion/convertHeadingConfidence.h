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
  asn1/raw/cam_en302637_2/CAM-PDU-Descriptions.asn \
  asn1/raw/cam_en302637_2/cdd/ITS-Container.asn \
  -t \
  cam \
  -o \
  etsi_its_conversion/etsi_its_cam_conversion/include/etsi_its_cam_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
HeadingConfidence ::= INTEGER {equalOrWithinZeroPointOneDegree (1), equalOrWithinOneDegree (10), outOfRange(126), unavailable(127)} (1..127)
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_cam_coding/cam_HeadingConfidence.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/HeadingConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/heading_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_HeadingConfidence(const cam_HeadingConfidence_t& in, cam_msgs::HeadingConfidence& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HeadingConfidence(const cam_msgs::HeadingConfidence& in, cam_HeadingConfidence_t& out) {
  memset(&out, 0, sizeof(cam_HeadingConfidence_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
