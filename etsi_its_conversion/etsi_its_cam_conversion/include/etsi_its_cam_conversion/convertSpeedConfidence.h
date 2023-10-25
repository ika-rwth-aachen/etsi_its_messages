#pragma once

#include <etsi_its_cam_coding/SpeedConfidence.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SpeedConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/speed_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SpeedConfidence(const SpeedConfidence_t& in, cam_msgs::SpeedConfidence& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SpeedConfidence(const cam_msgs::SpeedConfidence& in, SpeedConfidence_t& out) {

  memset(&out, 0, sizeof(SpeedConfidence_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}