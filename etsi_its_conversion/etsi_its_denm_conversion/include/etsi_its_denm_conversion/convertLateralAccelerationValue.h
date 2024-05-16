//// INTEGER LateralAccelerationValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/LateralAccelerationValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/LateralAccelerationValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/lateral_acceleration_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_LateralAccelerationValue(const LateralAccelerationValue_t& in, denm_msgs::LateralAccelerationValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_LateralAccelerationValue(const denm_msgs::LateralAccelerationValue& in, LateralAccelerationValue_t& out) {
  memset(&out, 0, sizeof(LateralAccelerationValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
