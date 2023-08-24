#pragma once

#include <etsi_its_denm_coding/PosLonCarr.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/pos_lon_carr.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PosLonCarr.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PosLonCarr(const PosLonCarr_t& in, denm_msgs::PosLonCarr& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_PosLonCarr(const denm_msgs::PosLonCarr& in, PosLonCarr_t& out) {

  memset(&out, 0, sizeof(PosLonCarr_t));
  toStruct_INTEGER(in.value, out);
}

}