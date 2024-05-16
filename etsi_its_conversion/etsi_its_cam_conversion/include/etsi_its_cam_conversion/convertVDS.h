//// IA5String VDS


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/VDS.h>
#include <etsi_its_cam_coding/IA5String.h>
#include <etsi_its_primitives_conversion/convertIA5String.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VDS.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vds.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VDS(const VDS_t& in, cam_msgs::VDS& out) {
  etsi_its_primitives_conversion::toRos_IA5String(in, out.value);
}

void toStruct_VDS(const cam_msgs::VDS& in, VDS_t& out) {
  memset(&out, 0, sizeof(VDS_t));

  etsi_its_primitives_conversion::toStruct_IA5String(in.value, out);
}

}
