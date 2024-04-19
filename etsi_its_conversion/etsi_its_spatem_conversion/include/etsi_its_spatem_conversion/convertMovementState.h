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

#include <etsi_its_spatem_coding/MovementState.h>
#include <etsi_its_spatem_conversion/convertDescriptiveName.h>
#include <etsi_its_spatem_conversion/convertSignalGroupID.h>
#include <etsi_its_spatem_conversion/convertMovementEventList.h>
#include <etsi_its_spatem_conversion/convertManeuverAssistList.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/MovementState.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/movement_state.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_MovementState(const MovementState_t& in, spatem_msgs::MovementState& out) {

  if (in.movementName) {
    toRos_DescriptiveName(*in.movementName, out.movement_name);
    out.movement_name_is_present = true;
  }

  toRos_SignalGroupID(in.signalGroup, out.signal_group);
  toRos_MovementEventList(in.state_time_speed, out.state_time_speed);
  if (in.maneuverAssistList) {
    toRos_ManeuverAssistList(*in.maneuverAssistList, out.maneuver_assist_list);
    out.maneuver_assist_list_is_present = true;
  }

  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_MovementState(const spatem_msgs::MovementState& in, MovementState_t& out) {

  memset(&out, 0, sizeof(MovementState_t));

  if (in.movement_name_is_present) {
    DescriptiveName_t movement_name;
    toStruct_DescriptiveName(in.movement_name, movement_name);
    out.movementName = new DescriptiveName_t(movement_name);
  }

  toStruct_SignalGroupID(in.signal_group, out.signalGroup);
  toStruct_MovementEventList(in.state_time_speed, out.state_time_speed);
  if (in.maneuver_assist_list_is_present) {
    ManeuverAssistList_t maneuver_assist_list;
    toStruct_ManeuverAssistList(in.maneuver_assist_list, maneuver_assist_list);
    out.maneuverAssistList = new ManeuverAssistList_t(maneuver_assist_list);
  }

  if (in.regional_is_present) {
    RegionalExtension_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_t(regional);
  }

}

}