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

#include <etsi_its_mapem_coding/HeadingConfidenceDSRC.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/HeadingConfidenceDSRC.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/heading_confidence_dsrc.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_HeadingConfidenceDSRC(const HeadingConfidenceDSRC_t& in, mapem_msgs::HeadingConfidenceDSRC& out) {

  out.value = in;
}

void toStruct_HeadingConfidenceDSRC(const mapem_msgs::HeadingConfidenceDSRC& in, HeadingConfidenceDSRC_t& out) {

  memset(&out, 0, sizeof(HeadingConfidenceDSRC_t));
  out = in.value;
}

}