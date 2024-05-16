//// INTEGER PtActivationType


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PtActivationType.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PtActivationType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/pt_activation_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PtActivationType(const PtActivationType_t& in, denm_msgs::PtActivationType& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PtActivationType(const denm_msgs::PtActivationType& in, PtActivationType_t& out) {
  memset(&out, 0, sizeof(PtActivationType_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
