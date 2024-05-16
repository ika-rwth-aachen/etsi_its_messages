//// INTEGER SemiAxisLength


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/SemiAxisLength.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SemiAxisLength.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/semi_axis_length.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SemiAxisLength(const SemiAxisLength_t& in, cam_msgs::SemiAxisLength& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SemiAxisLength(const cam_msgs::SemiAxisLength& in, SemiAxisLength_t& out) {
  memset(&out, 0, sizeof(SemiAxisLength_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
