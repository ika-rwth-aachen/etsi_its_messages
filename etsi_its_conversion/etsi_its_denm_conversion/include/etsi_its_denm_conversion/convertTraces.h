#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/Traces.h>
#include <etsi_its_denm_coding/PathHistory.h>
#include <etsi_its_denm_conversion/convertPathHistory.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/path_history.hpp>
#include <etsi_its_denm_msgs/msg/traces.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PathHistory.h>
#include <etsi_its_denm_msgs/Traces.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_Traces(const Traces_t& in, denm_msgs::Traces& out) {

  for (int i = 0; i < in.list.count; i++) {
    denm_msgs::PathHistory array;
    toRos_PathHistory(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_Traces(const denm_msgs::Traces& in, Traces_t& out) {
    
  memset(&out, 0, sizeof(Traces_t));

  for (int i = 0; i < in.array.size(); i++) {
    PathHistory_t array;
    toStruct_PathHistory(in.array[i], array);
    PathHistory_t* array_ptr = new PathHistory_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}