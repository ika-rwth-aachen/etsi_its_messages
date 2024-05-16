//// BIT-STRING AccelerationControl


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/AccelerationControl.h>
#include <etsi_its_denm_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/AccelerationControl.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/acceleration_control.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_AccelerationControl(const AccelerationControl_t& in, denm_msgs::AccelerationControl& out) {
  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_AccelerationControl(const denm_msgs::AccelerationControl& in, AccelerationControl_t& out) {
  memset(&out, 0, sizeof(AccelerationControl_t));

  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}
