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

#include <etsi_its_spatem_coding/NodeAttributeSetXY.h>
#include <etsi_its_spatem_conversion/convertNodeAttributeXYList.h>
#include <etsi_its_spatem_conversion/convertSegmentAttributeXYList.h>
#include <etsi_its_spatem_conversion/convertSegmentAttributeXYList.h>
#include <etsi_its_spatem_conversion/convertLaneDataAttributeList.h>
#include <etsi_its_spatem_conversion/convertOffsetB10.h>
#include <etsi_its_spatem_conversion/convertOffsetB10.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/NodeAttributeSetXY.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/node_attribute_set_x_y.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_NodeAttributeSetXY(const NodeAttributeSetXY_t& in, spatem_msgs::NodeAttributeSetXY& out) {

  if (in.localNode) {
    toRos_NodeAttributeXYList(*in.localNode, out.local_node);
    out.local_node_is_present = true;
  }

  if (in.disabled) {
    toRos_SegmentAttributeXYList(*in.disabled, out.disabled);
    out.disabled_is_present = true;
  }

  if (in.enabled) {
    toRos_SegmentAttributeXYList(*in.enabled, out.enabled);
    out.enabled_is_present = true;
  }

  if (in.data) {
    toRos_LaneDataAttributeList(*in.data, out.data);
    out.data_is_present = true;
  }

  if (in.dWidth) {
    toRos_OffsetB10(*in.dWidth, out.d_width);
    out.d_width_is_present = true;
  }

  if (in.dElevation) {
    toRos_OffsetB10(*in.dElevation, out.d_elevation);
    out.d_elevation_is_present = true;
  }

  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_NodeAttributeSetXY(const spatem_msgs::NodeAttributeSetXY& in, NodeAttributeSetXY_t& out) {

  memset(&out, 0, sizeof(NodeAttributeSetXY_t));

  if (in.local_node_is_present) {
    NodeAttributeXYList_t local_node;
    toStruct_NodeAttributeXYList(in.local_node, local_node);
    out.localNode = new NodeAttributeXYList_t(local_node);
  }

  if (in.disabled_is_present) {
    SegmentAttributeXYList_t disabled;
    toStruct_SegmentAttributeXYList(in.disabled, disabled);
    out.disabled = new SegmentAttributeXYList_t(disabled);
  }

  if (in.enabled_is_present) {
    SegmentAttributeXYList_t enabled;
    toStruct_SegmentAttributeXYList(in.enabled, enabled);
    out.enabled = new SegmentAttributeXYList_t(enabled);
  }

  if (in.data_is_present) {
    LaneDataAttributeList_t data;
    toStruct_LaneDataAttributeList(in.data, data);
    out.data = new LaneDataAttributeList_t(data);
  }

  if (in.d_width_is_present) {
    OffsetB10_t d_width;
    toStruct_OffsetB10(in.d_width, d_width);
    out.dWidth = new OffsetB10_t(d_width);
  }

  if (in.d_elevation_is_present) {
    OffsetB10_t d_elevation;
    toStruct_OffsetB10(in.d_elevation, d_elevation);
    out.dElevation = new OffsetB10_t(d_elevation);
  }

  if (in.regional_is_present) {
    RegionalExtension_364P0_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_364P0_t(regional);
  }

}

}