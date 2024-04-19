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

#include <etsi_its_mapem_coding/IntersectionGeometry.h>
#include <etsi_its_mapem_conversion/convertDescriptiveName.h>
#include <etsi_its_mapem_conversion/convertIntersectionReferenceID.h>
#include <etsi_its_mapem_conversion/convertMsgCount.h>
#include <etsi_its_mapem_conversion/convertPosition3D.h>
#include <etsi_its_mapem_conversion/convertLaneWidth.h>
#include <etsi_its_mapem_conversion/convertSpeedLimitList.h>
#include <etsi_its_mapem_conversion/convertLaneList.h>
#include <etsi_its_mapem_conversion/convertPreemptPriorityList.h>
#include <etsi_its_mapem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/IntersectionGeometry.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/intersection_geometry.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_IntersectionGeometry(const IntersectionGeometry_t& in, mapem_msgs::IntersectionGeometry& out) {

  if (in.name) {
    toRos_DescriptiveName(*in.name, out.name);
    out.name_is_present = true;
  }

  toRos_IntersectionReferenceID(in.id, out.id);
  toRos_MsgCount(in.revision, out.revision);
  toRos_Position3D(in.refPoint, out.ref_point);
  if (in.laneWidth) {
    toRos_LaneWidth(*in.laneWidth, out.lane_width);
    out.lane_width_is_present = true;
  }

  if (in.speedLimits) {
    toRos_SpeedLimitList(*in.speedLimits, out.speed_limits);
    out.speed_limits_is_present = true;
  }

  toRos_LaneList(in.laneSet, out.lane_set);
  if (in.preemptPriorityData) {
    toRos_PreemptPriorityList(*in.preemptPriorityData, out.preempt_priority_data);
    out.preempt_priority_data_is_present = true;
  }

  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_IntersectionGeometry(const mapem_msgs::IntersectionGeometry& in, IntersectionGeometry_t& out) {

  memset(&out, 0, sizeof(IntersectionGeometry_t));

  if (in.name_is_present) {
    DescriptiveName_t name;
    toStruct_DescriptiveName(in.name, name);
    out.name = new DescriptiveName_t(name);
  }

  toStruct_IntersectionReferenceID(in.id, out.id);
  toStruct_MsgCount(in.revision, out.revision);
  toStruct_Position3D(in.ref_point, out.refPoint);
  if (in.lane_width_is_present) {
    LaneWidth_t lane_width;
    toStruct_LaneWidth(in.lane_width, lane_width);
    out.laneWidth = new LaneWidth_t(lane_width);
  }

  if (in.speed_limits_is_present) {
    SpeedLimitList_t speed_limits;
    toStruct_SpeedLimitList(in.speed_limits, speed_limits);
    out.speedLimits = new SpeedLimitList_t(speed_limits);
  }

  toStruct_LaneList(in.lane_set, out.laneSet);
  if (in.preempt_priority_data_is_present) {
    PreemptPriorityList_t preempt_priority_data;
    toStruct_PreemptPriorityList(in.preempt_priority_data, preempt_priority_data);
    out.preemptPriorityData = new PreemptPriorityList_t(preempt_priority_data);
  }

  if (in.regional_is_present) {
    RegionalExtension_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_t(regional);
  }

}

}