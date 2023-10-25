#pragma once

#include <etsi_its_cam_coding/PerformanceClass.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PerformanceClass.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/performance_class.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PerformanceClass(const PerformanceClass_t& in, cam_msgs::PerformanceClass& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PerformanceClass(const cam_msgs::PerformanceClass& in, PerformanceClass_t& out) {

  memset(&out, 0, sizeof(PerformanceClass_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}