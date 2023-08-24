#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/EventHistory.h>
#include <etsi_its_denm_coding/EventPoint.h>
#include <etsi_its_denm_conversion/convertEventPoint.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/event_point.hpp>
#include <etsi_its_denm_msgs/msg/event_history.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/EventPoint.h>
#include <etsi_its_denm_msgs/EventHistory.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_EventHistory(const EventHistory_t& in, denm_msgs::EventHistory& out) {

  for (int i = 0; i < in.list.count; i++) {
    denm_msgs::EventPoint array;
    toRos_EventPoint(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_EventHistory(const denm_msgs::EventHistory& in, EventHistory_t& out) {
    
  memset(&out, 0, sizeof(EventHistory_t));

  for (int i = 0; i < in.array.size(); i++) {
    EventPoint_t array;
    toStruct_EventPoint(in.array[i], array);
    EventPoint_t* array_ptr = new EventPoint_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}