//// BIT-STRING SpecialTransportType


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/SpecialTransportType.h>
#include <etsi_its_denm_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SpecialTransportType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/special_transport_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SpecialTransportType(const SpecialTransportType_t& in, denm_msgs::SpecialTransportType& out) {
  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_SpecialTransportType(const denm_msgs::SpecialTransportType& in, SpecialTransportType_t& out) {
  memset(&out, 0, sizeof(SpecialTransportType_t));

  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}
