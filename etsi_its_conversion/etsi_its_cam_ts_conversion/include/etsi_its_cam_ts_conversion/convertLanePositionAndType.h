/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro

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

#include <etsi_its_cam_ts_coding/cam_ts_LanePositionAndType.h>
#include <etsi_its_cam_ts_conversion/convertDirection.h>
#include <etsi_its_cam_ts_conversion/convertLanePosition.h>
#include <etsi_its_cam_ts_conversion/convertLaneType.h>
#ifdef ROS1
#include <etsi_its_cam_ts_msgs/LanePositionAndType.h>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs;
#else
#include <etsi_its_cam_ts_msgs/msg/lane_position_and_type.hpp>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs::msg;
#endif


namespace etsi_its_cam_ts_conversion {

void toRos_LanePositionAndType(const cam_ts_LanePositionAndType_t& in, cam_ts_msgs::LanePositionAndType& out) {
  toRos_LanePosition(in.transversalPosition, out.transversal_position);
  toRos_LaneType(in.laneType, out.lane_type);
  toRos_Direction(in.direction, out.direction);
}

void toStruct_LanePositionAndType(const cam_ts_msgs::LanePositionAndType& in, cam_ts_LanePositionAndType_t& out) {
  memset(&out, 0, sizeof(cam_ts_LanePositionAndType_t));

  toStruct_LanePosition(in.transversal_position, out.transversalPosition);
  toStruct_LaneType(in.lane_type, out.laneType);
  toStruct_Direction(in.direction, out.direction);
}

}
