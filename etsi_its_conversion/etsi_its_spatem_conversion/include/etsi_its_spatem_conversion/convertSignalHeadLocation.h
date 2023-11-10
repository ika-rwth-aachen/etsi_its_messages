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

#include <etsi_its_spatem_coding/SignalHeadLocation.h>
#include <etsi_its_spatem_conversion/convertNodeOffsetPointXY.h>
#include <etsi_its_spatem_conversion/convertDeltaAltitude.h>
#include <etsi_its_spatem_conversion/convertSignalGroupID.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/SignalHeadLocation.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/signal_head_location.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_SignalHeadLocation(const SignalHeadLocation_t& in, spatem_msgs::SignalHeadLocation& out) {

  toRos_NodeOffsetPointXY(in.nodeXY, out.node_x_y);
  toRos_DeltaAltitude(in.nodeZ, out.node_z);
  toRos_SignalGroupID(in.signalGroupID, out.signal_group_id);
}

void toStruct_SignalHeadLocation(const spatem_msgs::SignalHeadLocation& in, SignalHeadLocation_t& out) {

  memset(&out, 0, sizeof(SignalHeadLocation_t));

  toStruct_NodeOffsetPointXY(in.node_x_y, out.nodeXY);
  toStruct_DeltaAltitude(in.node_z, out.nodeZ);
  toStruct_SignalGroupID(in.signal_group_id, out.signalGroupID);
}

}