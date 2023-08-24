#pragma once

#include <etsi_its_denm_coding/VDS.h>
#include <etsi_its_denm_conversion/primitives/convertIA5String.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/vds.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/VDS.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_VDS(const VDS_t& in, denm_msgs::VDS& out) {

  toRos_IA5String(in, out.value);
}

void toStruct_VDS(const denm_msgs::VDS& in, VDS_t& out) {

  memset(&out, 0, sizeof(VDS_t));
  toStruct_IA5String(in.value, out);
}

}