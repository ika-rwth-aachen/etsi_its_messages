//// SEQUENCE-OF ReferenceDenms


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/ReferenceDenms.h>
#include <etsi_its_denm_conversion/convertReferenceDenms.h>
#include <etsi_its_denm_conversion/convertActionID.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/ReferenceDenms.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/reference_denms.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_ReferenceDenms(const ReferenceDenms_t& in, denm_msgs::ReferenceDenms& out) {
  for (int i = 0; i < in.list.count; ++i) {
    denm_msgs::ActionID el;
    toRos_ActionID(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_ReferenceDenms(const denm_msgs::ReferenceDenms& in, ReferenceDenms_t& out) {
  memset(&out, 0, sizeof(ReferenceDenms_t));

  for (int i = 0; i < in.array.size(); ++i) {
    ActionID_t* el = (ActionID_t*) calloc(1, sizeof(ActionID_t));
    toStruct_ActionID(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
