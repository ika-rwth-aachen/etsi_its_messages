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

#include <etsi_its_mapem_ts_coding/mapem_ts_RoadSegmentID.h>
#include <etsi_its_mapem_ts_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_mapem_ts_msgs/RoadSegmentID.h>
namespace mapem_ts_msgs = etsi_its_mapem_ts_msgs;
#else
#include <etsi_its_mapem_ts_msgs/msg/road_segment_id.hpp>
namespace mapem_ts_msgs = etsi_its_mapem_ts_msgs::msg;
#endif


namespace etsi_its_mapem_ts_conversion {

void toRos_RoadSegmentID(const mapem_ts_RoadSegmentID_t& in, mapem_ts_msgs::RoadSegmentID& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_RoadSegmentID(const mapem_ts_msgs::RoadSegmentID& in, mapem_ts_RoadSegmentID_t& out) {
  memset(&out, 0, sizeof(mapem_ts_RoadSegmentID_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
