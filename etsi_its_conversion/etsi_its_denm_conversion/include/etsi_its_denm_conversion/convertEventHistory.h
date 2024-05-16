//// SEQUENCE-OF EventHistory


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/EventHistory.h>
#include <etsi_its_denm_conversion/convertEventHistory.h>
#include <etsi_its_denm_conversion/convertEventPoint.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/EventHistory.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/event_history.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_EventHistory(const EventHistory_t& in, denm_msgs::EventHistory& out) {
  for (int i = 0; i < in.list.count; ++i) {
    denm_msgs::EventPoint el;
    toRos_EventPoint(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_EventHistory(const denm_msgs::EventHistory& in, EventHistory_t& out) {
  memset(&out, 0, sizeof(EventHistory_t));

  for (int i = 0; i < in.array.size(); ++i) {
    EventPoint_t* el = (EventPoint_t*) calloc(1, sizeof(EventPoint_t));
    toStruct_EventPoint(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
