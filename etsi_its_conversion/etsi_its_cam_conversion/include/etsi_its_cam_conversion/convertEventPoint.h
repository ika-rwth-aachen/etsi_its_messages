//// SEQUENCE EventPoint


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/EventPoint.h>
#include <etsi_its_cam_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertPathDeltaTime.h>
#include <etsi_its_cam_conversion/convertInformationQuality.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/EventPoint.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/event_point.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_EventPoint(const EventPoint_t& in, cam_msgs::EventPoint& out) {
  toRos_DeltaReferencePosition(in.eventPosition, out.event_position);
  if (in.eventDeltaTime) {
    toRos_PathDeltaTime(*in.eventDeltaTime, out.event_delta_time);
    out.event_delta_time_is_present = true;
  }
  toRos_InformationQuality(in.informationQuality, out.information_quality);
}

void toStruct_EventPoint(const cam_msgs::EventPoint& in, EventPoint_t& out) {
  memset(&out, 0, sizeof(EventPoint_t));

  toStruct_DeltaReferencePosition(in.event_position, out.eventPosition);
  if (in.event_delta_time_is_present) {
    out.eventDeltaTime = (PathDeltaTime_t*) calloc(1, sizeof(PathDeltaTime_t));
    toStruct_PathDeltaTime(in.event_delta_time, *out.eventDeltaTime);
  }
  toStruct_InformationQuality(in.information_quality, out.informationQuality);
}

}
