//// INTEGER PosLonCarr


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PosLonCarr.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PosLonCarr.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/pos_lon_carr.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PosLonCarr(const PosLonCarr_t& in, denm_msgs::PosLonCarr& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PosLonCarr(const denm_msgs::PosLonCarr& in, PosLonCarr_t& out) {
  memset(&out, 0, sizeof(PosLonCarr_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
