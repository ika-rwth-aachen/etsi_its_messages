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

#include <etsi_its_spatem_coding/GenericLane.h>
#include <etsi_its_spatem_conversion/convertLaneID.h>
#include <etsi_its_spatem_conversion/convertDescriptiveName.h>
#include <etsi_its_spatem_conversion/convertApproachID.h>
#include <etsi_its_spatem_conversion/convertApproachID.h>
#include <etsi_its_spatem_conversion/convertLaneAttributes.h>
#include <etsi_its_spatem_conversion/convertAllowedManeuvers.h>
#include <etsi_its_spatem_conversion/convertNodeListXY.h>
#include <etsi_its_spatem_conversion/convertConnectsToList.h>
#include <etsi_its_spatem_conversion/convertOverlayLaneList.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/GenericLane.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/generic_lane.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_GenericLane(const GenericLane_t& in, spatem_msgs::GenericLane& out) {

  toRos_LaneID(in.laneID, out.lane_id);
  if (in.name) {
    toRos_DescriptiveName(*in.name, out.name);
    out.name_is_present = true;
  }

  if (in.ingressApproach) {
    toRos_ApproachID(*in.ingressApproach, out.ingress_approach);
    out.ingress_approach_is_present = true;
  }

  if (in.egressApproach) {
    toRos_ApproachID(*in.egressApproach, out.egress_approach);
    out.egress_approach_is_present = true;
  }

  toRos_LaneAttributes(in.laneAttributes, out.lane_attributes);
  if (in.maneuvers) {
    toRos_AllowedManeuvers(*in.maneuvers, out.maneuvers);
    out.maneuvers_is_present = true;
  }

  toRos_NodeListXY(in.nodeList, out.node_list);
  if (in.connectsTo) {
    toRos_ConnectsToList(*in.connectsTo, out.connects_to);
    out.connects_to_is_present = true;
  }

  if (in.overlays) {
    toRos_OverlayLaneList(*in.overlays, out.overlays);
    out.overlays_is_present = true;
  }

  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_GenericLane(const spatem_msgs::GenericLane& in, GenericLane_t& out) {

  memset(&out, 0, sizeof(GenericLane_t));

  toStruct_LaneID(in.lane_id, out.laneID);
  if (in.name_is_present) {
    DescriptiveName_t name;
    toStruct_DescriptiveName(in.name, name);
    out.name = new DescriptiveName_t(name);
  }

  if (in.ingress_approach_is_present) {
    ApproachID_t ingress_approach;
    toStruct_ApproachID(in.ingress_approach, ingress_approach);
    out.ingressApproach = new ApproachID_t(ingress_approach);
  }

  if (in.egress_approach_is_present) {
    ApproachID_t egress_approach;
    toStruct_ApproachID(in.egress_approach, egress_approach);
    out.egressApproach = new ApproachID_t(egress_approach);
  }

  toStruct_LaneAttributes(in.lane_attributes, out.laneAttributes);
  if (in.maneuvers_is_present) {
    AllowedManeuvers_t maneuvers;
    toStruct_AllowedManeuvers(in.maneuvers, maneuvers);
    out.maneuvers = new AllowedManeuvers_t(maneuvers);
  }

  toStruct_NodeListXY(in.node_list, out.nodeList);
  if (in.connects_to_is_present) {
    ConnectsToList_t connects_to;
    toStruct_ConnectsToList(in.connects_to, connects_to);
    out.connectsTo = new ConnectsToList_t(connects_to);
  }

  if (in.overlays_is_present) {
    OverlayLaneList_t overlays;
    toStruct_OverlayLaneList(in.overlays, overlays);
    out.overlays = new OverlayLaneList_t(overlays);
  }

  if (in.regional_is_present) {
    RegionalExtension_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_t(regional);
  }

}

}