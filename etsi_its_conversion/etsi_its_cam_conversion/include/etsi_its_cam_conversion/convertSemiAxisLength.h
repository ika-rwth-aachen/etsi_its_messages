#pragma once

#include <etsi_its_cam_coding/SemiAxisLength.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SemiAxisLength.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/semi_axis_length.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SemiAxisLength(const SemiAxisLength_t& in, cam_msgs::SemiAxisLength& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SemiAxisLength(const cam_msgs::SemiAxisLength& in, SemiAxisLength_t& out) {

  memset(&out, 0, sizeof(SemiAxisLength_t));
  toStruct_INTEGER(in.value, out);
}

}