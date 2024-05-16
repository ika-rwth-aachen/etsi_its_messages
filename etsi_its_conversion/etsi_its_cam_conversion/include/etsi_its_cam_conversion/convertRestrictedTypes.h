//// SEQUENCE-OF RestrictedTypes


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/RestrictedTypes.h>
#include <etsi_its_cam_conversion/convertRestrictedTypes.h>
#include <etsi_its_cam_conversion/convertStationType.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/RestrictedTypes.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/restricted_types.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RestrictedTypes(const RestrictedTypes_t& in, cam_msgs::RestrictedTypes& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cam_msgs::StationType el;
    toRos_StationType(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_RestrictedTypes(const cam_msgs::RestrictedTypes& in, RestrictedTypes_t& out) {
  memset(&out, 0, sizeof(RestrictedTypes_t));

  for (int i = 0; i < in.array.size(); ++i) {
    StationType_t* el = (StationType_t*) calloc(1, sizeof(StationType_t));
    toStruct_StationType(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
