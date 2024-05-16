//// INTEGER TurningRadius


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/TurningRadius.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/TurningRadius.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/turning_radius.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_TurningRadius(const TurningRadius_t& in, cam_msgs::TurningRadius& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_TurningRadius(const cam_msgs::TurningRadius& in, TurningRadius_t& out) {
  memset(&out, 0, sizeof(TurningRadius_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
