//// INTEGER PosPillar


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PosPillar.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PosPillar.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/pos_pillar.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PosPillar(const PosPillar_t& in, cam_msgs::PosPillar& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PosPillar(const cam_msgs::PosPillar& in, PosPillar_t& out) {
  memset(&out, 0, sizeof(PosPillar_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
