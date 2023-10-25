#pragma once

#include <etsi_its_denm_coding/TransmissionInterval.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/TransmissionInterval.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/transmission_interval.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_TransmissionInterval(const TransmissionInterval_t& in, denm_msgs::TransmissionInterval& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_TransmissionInterval(const denm_msgs::TransmissionInterval& in, TransmissionInterval_t& out) {

  memset(&out, 0, sizeof(TransmissionInterval_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}