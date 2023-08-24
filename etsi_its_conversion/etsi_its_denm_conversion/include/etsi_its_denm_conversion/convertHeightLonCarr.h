#pragma once

#include <etsi_its_denm_coding/HeightLonCarr.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/height_lon_carr.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/HeightLonCarr.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_HeightLonCarr(const HeightLonCarr_t& in, denm_msgs::HeightLonCarr& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_HeightLonCarr(const denm_msgs::HeightLonCarr& in, HeightLonCarr_t& out) {

  memset(&out, 0, sizeof(HeightLonCarr_t));
  toStruct_INTEGER(in.value, out);
}

}