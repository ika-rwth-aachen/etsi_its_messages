#pragma once

#include <etsi_its_denm_coding/LocationContainer.h>
#include <etsi_its_denm_conversion/convertSpeed.h>
#include <etsi_its_denm_conversion/convertHeading.h>
#include <etsi_its_denm_conversion/convertTraces.h>
#include <etsi_its_denm_conversion/convertRoadType.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/location_container.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/LocationContainer.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_LocationContainer(const LocationContainer_t& in, denm_msgs::LocationContainer& out) {

  if (in.eventSpeed) {
    toRos_Speed(*in.eventSpeed, out.event_speed);
    out.event_speed_is_present = true;
  }

  if (in.eventPositionHeading) {
    toRos_Heading(*in.eventPositionHeading, out.event_position_heading);
    out.event_position_heading_is_present = true;
  }

  toRos_Traces(in.traces, out.traces);
  if (in.roadType) {
    toRos_RoadType(*in.roadType, out.road_type);
    out.road_type_is_present = true;
  }

}

void toStruct_LocationContainer(const denm_msgs::LocationContainer& in, LocationContainer_t& out) {
    
  memset(&out, 0, sizeof(LocationContainer_t));

  if (in.event_speed_is_present) {
    Speed_t event_speed;
    toStruct_Speed(in.event_speed, event_speed);
    out.eventSpeed = new Speed_t(event_speed);
  }

  if (in.event_position_heading_is_present) {
    Heading_t event_position_heading;
    toStruct_Heading(in.event_position_heading, event_position_heading);
    out.eventPositionHeading = new Heading_t(event_position_heading);
  }

  toStruct_Traces(in.traces, out.traces);
  if (in.road_type_is_present) {
    RoadType_t road_type;
    toStruct_RoadType(in.road_type, road_type);
    out.roadType = new RoadType_t(road_type);
  }

}

}