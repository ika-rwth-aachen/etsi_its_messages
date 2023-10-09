#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/ItineraryPath.h>
#include <etsi_its_denm_coding/ReferencePosition.h>
#include <etsi_its_denm_conversion/convertReferencePosition.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/ReferencePosition.h>
#include <etsi_its_denm_msgs/ItineraryPath.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/reference_position.hpp>
#include <etsi_its_denm_msgs/msg/itinerary_path.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_ItineraryPath(const ItineraryPath_t& in, denm_msgs::ItineraryPath& out) {

  for (int i = 0; i < in.list.count; i++) {
    denm_msgs::ReferencePosition array;
    toRos_ReferencePosition(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_ItineraryPath(const denm_msgs::ItineraryPath& in, ItineraryPath_t& out) {

  memset(&out, 0, sizeof(ItineraryPath_t));

  for (int i = 0; i < in.array.size(); i++) {
    ReferencePosition_t array;
    toStruct_ReferencePosition(in.array[i], array);
    ReferencePosition_t* array_ptr = new ReferencePosition_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}