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

#include <etsi_its_mapem_coding/LaneAttributes.h>
#include <etsi_its_mapem_conversion/convertLaneDirection.h>
#include <etsi_its_mapem_conversion/convertLaneSharing.h>
#include <etsi_its_mapem_conversion/convertLaneTypeAttributes.h>
#include <etsi_its_mapem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/LaneAttributes.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/lane_attributes.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_LaneAttributes(const LaneAttributes_t& in, mapem_msgs::LaneAttributes& out) {

  toRos_LaneDirection(in.directionalUse, out.directional_use);
  toRos_LaneSharing(in.sharedWith, out.shared_with);
  toRos_LaneTypeAttributes(in.laneType, out.lane_type);
  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_LaneAttributes(const mapem_msgs::LaneAttributes& in, LaneAttributes_t& out) {

  memset(&out, 0, sizeof(LaneAttributes_t));

  toStruct_LaneDirection(in.directional_use, out.directionalUse);
  toStruct_LaneSharing(in.shared_with, out.sharedWith);
  toStruct_LaneTypeAttributes(in.lane_type, out.laneType);
  if (in.regional_is_present) {
    RegionalExtension_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_t(regional);
  }

}

}