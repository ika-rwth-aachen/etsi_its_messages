//// INTEGER LongitudinalAccelerationValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/LongitudinalAccelerationValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/LongitudinalAccelerationValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/longitudinal_acceleration_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_LongitudinalAccelerationValue(const LongitudinalAccelerationValue_t& in, denm_msgs::LongitudinalAccelerationValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_LongitudinalAccelerationValue(const denm_msgs::LongitudinalAccelerationValue& in, LongitudinalAccelerationValue_t& out) {
  memset(&out, 0, sizeof(LongitudinalAccelerationValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
