//// SEQUENCE PtActivation


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PtActivation.h>
#include <etsi_its_denm_conversion/convertPtActivationType.h>
#include <etsi_its_denm_conversion/convertPtActivationData.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PtActivation.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/pt_activation.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PtActivation(const PtActivation_t& in, denm_msgs::PtActivation& out) {
  toRos_PtActivationType(in.ptActivationType, out.pt_activation_type);
  toRos_PtActivationData(in.ptActivationData, out.pt_activation_data);
}

void toStruct_PtActivation(const denm_msgs::PtActivation& in, PtActivation_t& out) {
  memset(&out, 0, sizeof(PtActivation_t));

  toStruct_PtActivationType(in.pt_activation_type, out.ptActivationType);
  toStruct_PtActivationData(in.pt_activation_data, out.ptActivationData);
}

}
