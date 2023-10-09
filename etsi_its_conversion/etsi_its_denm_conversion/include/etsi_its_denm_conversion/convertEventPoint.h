#pragma once

#include <etsi_its_denm_coding/EventPoint.h>
#include <etsi_its_denm_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_denm_conversion/convertPathDeltaTime.h>
#include <etsi_its_denm_conversion/convertInformationQuality.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/EventPoint.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/event_point.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_EventPoint(const EventPoint_t& in, denm_msgs::EventPoint& out) {

  toRos_DeltaReferencePosition(in.eventPosition, out.event_position);
  if (in.eventDeltaTime) {
    toRos_PathDeltaTime(*in.eventDeltaTime, out.event_delta_time);
    out.event_delta_time_is_present = true;
  }

  toRos_InformationQuality(in.informationQuality, out.information_quality);
}

void toStruct_EventPoint(const denm_msgs::EventPoint& in, EventPoint_t& out) {

  memset(&out, 0, sizeof(EventPoint_t));

  toStruct_DeltaReferencePosition(in.event_position, out.eventPosition);
  if (in.event_delta_time_is_present) {
    PathDeltaTime_t event_delta_time;
    toStruct_PathDeltaTime(in.event_delta_time, event_delta_time);
    out.eventDeltaTime = new PathDeltaTime_t(event_delta_time);
  }

  toStruct_InformationQuality(in.information_quality, out.informationQuality);
}

}