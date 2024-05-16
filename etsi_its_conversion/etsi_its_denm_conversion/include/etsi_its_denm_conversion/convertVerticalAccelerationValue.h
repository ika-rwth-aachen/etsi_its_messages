//// INTEGER VerticalAccelerationValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/VerticalAccelerationValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/VerticalAccelerationValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vertical_acceleration_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VerticalAccelerationValue(const VerticalAccelerationValue_t& in, denm_msgs::VerticalAccelerationValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_VerticalAccelerationValue(const denm_msgs::VerticalAccelerationValue& in, VerticalAccelerationValue_t& out) {
  memset(&out, 0, sizeof(VerticalAccelerationValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
