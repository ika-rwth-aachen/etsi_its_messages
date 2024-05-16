//// INTEGER PerformanceClass


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PerformanceClass.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PerformanceClass.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/performance_class.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PerformanceClass(const PerformanceClass_t& in, denm_msgs::PerformanceClass& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PerformanceClass(const denm_msgs::PerformanceClass& in, PerformanceClass_t& out) {
  memset(&out, 0, sizeof(PerformanceClass_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
