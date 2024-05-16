//// SEQUENCE-OF ItineraryPath


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/ItineraryPath.h>
#include <etsi_its_cam_conversion/convertItineraryPath.h>
#include <etsi_its_cam_conversion/convertReferencePosition.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ItineraryPath.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/itinerary_path.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ItineraryPath(const ItineraryPath_t& in, cam_msgs::ItineraryPath& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cam_msgs::ReferencePosition el;
    toRos_ReferencePosition(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_ItineraryPath(const cam_msgs::ItineraryPath& in, ItineraryPath_t& out) {
  memset(&out, 0, sizeof(ItineraryPath_t));

  for (int i = 0; i < in.array.size(); ++i) {
    ReferencePosition_t* el = (ReferencePosition_t*) calloc(1, sizeof(ReferencePosition_t));
    toStruct_ReferencePosition(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
