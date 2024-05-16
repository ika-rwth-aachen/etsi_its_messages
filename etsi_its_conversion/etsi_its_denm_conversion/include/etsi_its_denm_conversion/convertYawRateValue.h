//// INTEGER YawRateValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/YawRateValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/YawRateValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/yaw_rate_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_YawRateValue(const YawRateValue_t& in, denm_msgs::YawRateValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_YawRateValue(const denm_msgs::YawRateValue& in, YawRateValue_t& out) {
  memset(&out, 0, sizeof(YawRateValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
