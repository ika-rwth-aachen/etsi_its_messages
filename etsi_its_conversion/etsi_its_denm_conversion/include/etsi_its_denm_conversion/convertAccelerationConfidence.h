//// INTEGER AccelerationConfidence


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/AccelerationConfidence.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/AccelerationConfidence.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/acceleration_confidence.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_AccelerationConfidence(const AccelerationConfidence_t& in, denm_msgs::AccelerationConfidence& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AccelerationConfidence(const denm_msgs::AccelerationConfidence& in, AccelerationConfidence_t& out) {
  memset(&out, 0, sizeof(AccelerationConfidence_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
