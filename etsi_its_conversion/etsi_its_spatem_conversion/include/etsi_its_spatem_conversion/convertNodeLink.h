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
#include <etsi_its_spatem_coding/NodeLink.h>
#include <etsi_its_spatem_coding/Node.h>
#include <etsi_its_spatem_conversion/convertNode.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/Node.h>
#include <etsi_its_spatem_msgs/NodeLink.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/node.hpp>
#include <etsi_its_spatem_msgs/msg/node_link.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_NodeLink(const NodeLink_t& in, spatem_msgs::NodeLink& out) {

  for (int i = 0; i < in.list.count; i++) {
    spatem_msgs::Node array;
    toRos_Node(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_NodeLink(const spatem_msgs::NodeLink& in, NodeLink_t& out) {

  memset(&out, 0, sizeof(NodeLink_t));

  for (int i = 0; i < in.array.size(); i++) {
    Node_t array;
    toStruct_Node(in.array[i], array);
    Node_t* array_ptr = new Node_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}