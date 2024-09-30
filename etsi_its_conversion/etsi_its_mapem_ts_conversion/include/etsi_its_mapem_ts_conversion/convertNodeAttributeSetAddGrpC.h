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

#include <etsi_its_mapem_ts_coding/mapem_ts_NodeAttributeSet-addGrpC.h>
#include <etsi_its_mapem_ts_conversion/convertNode.h>
#include <etsi_its_mapem_ts_conversion/convertNodeLink.h>
#include <etsi_its_mapem_ts_conversion/convertPtvRequestType.h>
#ifdef ROS1
#include <etsi_its_mapem_ts_msgs/NodeAttributeSetAddGrpC.h>
namespace mapem_ts_msgs = etsi_its_mapem_ts_msgs;
#else
#include <etsi_its_mapem_ts_msgs/msg/node_attribute_set_add_grp_c.hpp>
namespace mapem_ts_msgs = etsi_its_mapem_ts_msgs::msg;
#endif


namespace etsi_its_mapem_ts_conversion {

void toRos_NodeAttributeSetAddGrpC(const mapem_ts_NodeAttributeSet_addGrpC_t& in, mapem_ts_msgs::NodeAttributeSetAddGrpC& out) {
  if (in.ptvRequest) {
    toRos_PtvRequestType(*in.ptvRequest, out.ptv_request);
    out.ptv_request_is_present = true;
  }
  if (in.nodeLink) {
    toRos_NodeLink(*in.nodeLink, out.node_link);
    out.node_link_is_present = true;
  }
  if (in.node) {
    toRos_Node(*in.node, out.node);
    out.node_is_present = true;
  }
}

void toStruct_NodeAttributeSetAddGrpC(const mapem_ts_msgs::NodeAttributeSetAddGrpC& in, mapem_ts_NodeAttributeSet_addGrpC_t& out) {
  memset(&out, 0, sizeof(mapem_ts_NodeAttributeSet_addGrpC_t));

  if (in.ptv_request_is_present) {
    out.ptvRequest = (mapem_ts_PtvRequestType_t*) calloc(1, sizeof(mapem_ts_PtvRequestType_t));
    toStruct_PtvRequestType(in.ptv_request, *out.ptvRequest);
  }
  if (in.node_link_is_present) {
    out.nodeLink = (mapem_ts_NodeLink_t*) calloc(1, sizeof(mapem_ts_NodeLink_t));
    toStruct_NodeLink(in.node_link, *out.nodeLink);
  }
  if (in.node_is_present) {
    out.node = (mapem_ts_Node_t*) calloc(1, sizeof(mapem_ts_Node_t));
    toStruct_Node(in.node, *out.node);
  }
}

}
