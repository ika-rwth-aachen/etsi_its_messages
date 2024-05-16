//// SEQUENCE-OF EventHistory


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/EventHistory.h>
#include <etsi_its_cam_conversion/convertEventHistory.h>
#include <etsi_its_cam_conversion/convertEventPoint.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/EventHistory.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/event_history.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_EventHistory(const EventHistory_t& in, cam_msgs::EventHistory& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cam_msgs::EventPoint el;
    toRos_EventPoint(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_EventHistory(const cam_msgs::EventHistory& in, EventHistory_t& out) {
  memset(&out, 0, sizeof(EventHistory_t));

  for (int i = 0; i < in.array.size(); ++i) {
    EventPoint_t* el = (EventPoint_t*) calloc(1, sizeof(EventPoint_t));
    toStruct_EventPoint(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
