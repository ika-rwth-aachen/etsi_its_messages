//// INTEGER HeadingConfidence


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/HeadingConfidence.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/HeadingConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/heading_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_HeadingConfidence(const HeadingConfidence_t& in, cam_msgs::HeadingConfidence& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HeadingConfidence(const cam_msgs::HeadingConfidence& in, HeadingConfidence_t& out) {
  memset(&out, 0, sizeof(HeadingConfidence_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
