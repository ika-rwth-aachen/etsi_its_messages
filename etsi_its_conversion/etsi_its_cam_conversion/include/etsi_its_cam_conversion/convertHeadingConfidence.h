#pragma once

#include <etsi_its_cam_coding/HeadingConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/heading_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/HeadingConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_HeadingConfidence(const HeadingConfidence_t& in, cam_msgs::HeadingConfidence& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_HeadingConfidence(const cam_msgs::HeadingConfidence& in, HeadingConfidence_t& out) {

  memset(&out, 0, sizeof(HeadingConfidence_t));
  toStruct_INTEGER(in.value, out);
}

}