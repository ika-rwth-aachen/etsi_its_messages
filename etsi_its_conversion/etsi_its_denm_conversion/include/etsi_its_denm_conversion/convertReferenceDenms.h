#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/ReferenceDenms.h>
#include <etsi_its_denm_coding/ActionID.h>
#include <etsi_its_denm_conversion/convertActionID.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/action_id.hpp>
#include <etsi_its_denm_msgs/msg/reference_denms.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/ActionID.h>
#include <etsi_its_denm_msgs/ReferenceDenms.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_ReferenceDenms(const ReferenceDenms_t& in, denm_msgs::ReferenceDenms& out) {

  for (int i = 0; i < in.list.count; i++) {
    denm_msgs::ActionID array;
    toRos_ActionID(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_ReferenceDenms(const denm_msgs::ReferenceDenms& in, ReferenceDenms_t& out) {
    
  memset(&out, 0, sizeof(ReferenceDenms_t));

  for (int i = 0; i < in.array.size(); i++) {
    ActionID_t array;
    toStruct_ActionID(in.array[i], array);
    ActionID_t* array_ptr = new ActionID_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}