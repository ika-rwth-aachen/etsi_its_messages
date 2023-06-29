#pragma once

#include <etsi_its_cam_coding/PerformanceClass.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/PerformanceClass.h>


namespace etsi_its_cam_conversion {

void toRos_PerformanceClass(const PerformanceClass_t& in, etsi_its_cam_msgs::PerformanceClass& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_PerformanceClass(const etsi_its_cam_msgs::PerformanceClass& in, PerformanceClass_t& out) {
    
  memset(&out, 0, sizeof(PerformanceClass_t));
  toStruct_INTEGER(in.value, out);
}

}