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

#include <etsi_its_mapem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_mapem_coding/IntersectionStateList.h>
#include <etsi_its_mapem_coding/IntersectionState.h>
#include <etsi_its_mapem_conversion/convertIntersectionState.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/IntersectionState.h>
#include <etsi_its_mapem_msgs/IntersectionStateList.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/intersection_state.hpp>
#include <etsi_its_mapem_msgs/msg/intersection_state_list.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_IntersectionStateList(const IntersectionStateList_t& in, mapem_msgs::IntersectionStateList& out) {

  for (int i = 0; i < in.list.count; i++) {
    mapem_msgs::IntersectionState array;
    toRos_IntersectionState(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_IntersectionStateList(const mapem_msgs::IntersectionStateList& in, IntersectionStateList_t& out) {

  memset(&out, 0, sizeof(IntersectionStateList_t));

  for (int i = 0; i < in.array.size(); i++) {
    IntersectionState_t array;
    toStruct_IntersectionState(in.array[i], array);
    IntersectionState_t* array_ptr = new IntersectionState_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}