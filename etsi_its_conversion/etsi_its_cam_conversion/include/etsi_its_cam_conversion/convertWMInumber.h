//// IA5String WMInumber


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/WMInumber.h>
#include <etsi_its_cam_coding/IA5String.h>
#include <etsi_its_primitives_conversion/convertIA5String.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/WMInumber.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/wm_inumber.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_WMInumber(const WMInumber_t& in, cam_msgs::WMInumber& out) {
  etsi_its_primitives_conversion::toRos_IA5String(in, out.value);
}

void toStruct_WMInumber(const cam_msgs::WMInumber& in, WMInumber_t& out) {
  memset(&out, 0, sizeof(WMInumber_t));

  etsi_its_primitives_conversion::toStruct_IA5String(in.value, out);
}

}
