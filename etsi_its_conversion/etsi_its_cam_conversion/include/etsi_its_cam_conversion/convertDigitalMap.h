//// SEQUENCE-OF DigitalMap


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/DigitalMap.h>
#include <etsi_its_cam_conversion/convertDigitalMap.h>
#include <etsi_its_cam_conversion/convertReferencePosition.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/DigitalMap.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/digital_map.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_DigitalMap(const DigitalMap_t& in, cam_msgs::DigitalMap& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cam_msgs::ReferencePosition el;
    toRos_ReferencePosition(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_DigitalMap(const cam_msgs::DigitalMap& in, DigitalMap_t& out) {
  memset(&out, 0, sizeof(DigitalMap_t));

  for (int i = 0; i < in.array.size(); ++i) {
    ReferencePosition_t* el = (ReferencePosition_t*) calloc(1, sizeof(ReferencePosition_t));
    toStruct_ReferencePosition(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
