#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/RestrictedTypes.h>
#include <etsi_its_denm_coding/StationType.h>
#include <etsi_its_denm_conversion/convertStationType.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/StationType.h>
#include <etsi_its_denm_msgs/RestrictedTypes.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/station_type.hpp>
#include <etsi_its_denm_msgs/msg/restricted_types.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_RestrictedTypes(const RestrictedTypes_t& in, denm_msgs::RestrictedTypes& out) {

  for (int i = 0; i < in.list.count; i++) {
    denm_msgs::StationType array;
    toRos_StationType(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_RestrictedTypes(const denm_msgs::RestrictedTypes& in, RestrictedTypes_t& out) {

  memset(&out, 0, sizeof(RestrictedTypes_t));

  for (int i = 0; i < in.array.size(); i++) {
    StationType_t array;
    toStruct_StationType(in.array[i], array);
    StationType_t* array_ptr = new StationType_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}