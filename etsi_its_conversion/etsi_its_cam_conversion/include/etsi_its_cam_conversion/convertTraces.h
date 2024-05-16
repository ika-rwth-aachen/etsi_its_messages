//// SEQUENCE-OF Traces


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/Traces.h>
#include <etsi_its_cam_conversion/convertTraces.h>
#include <etsi_its_cam_conversion/convertPathHistory.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/Traces.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/traces.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_Traces(const Traces_t& in, cam_msgs::Traces& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cam_msgs::PathHistory el;
    toRos_PathHistory(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_Traces(const cam_msgs::Traces& in, Traces_t& out) {
  memset(&out, 0, sizeof(Traces_t));

  for (int i = 0; i < in.array.size(); ++i) {
    PathHistory_t* el = (PathHistory_t*) calloc(1, sizeof(PathHistory_t));
    toStruct_PathHistory(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
