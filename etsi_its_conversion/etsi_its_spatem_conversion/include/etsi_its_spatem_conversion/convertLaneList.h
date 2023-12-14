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

#include <stdexcept>

#include <etsi_its_spatem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_spatem_coding/LaneList.h>
#include <etsi_its_spatem_coding/GenericLane.h>
#include <etsi_its_spatem_conversion/convertGenericLane.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/GenericLane.h>
#include <etsi_its_spatem_msgs/LaneList.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/generic_lane.hpp>
#include <etsi_its_spatem_msgs/msg/lane_list.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_LaneList(const LaneList_t& in, spatem_msgs::LaneList& out) {

  for (int i = 0; i < in.list.count; i++) {
    spatem_msgs::GenericLane array;
    toRos_GenericLane(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_LaneList(const spatem_msgs::LaneList& in, LaneList_t& out) {

  memset(&out, 0, sizeof(LaneList_t));

  for (int i = 0; i < in.array.size(); i++) {
    GenericLane_t array;
    toStruct_GenericLane(in.array[i], array);
    GenericLane_t* array_ptr = new GenericLane_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}