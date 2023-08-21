#pragma once

#include <etsi_its_cam_coding/HeadingValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/heading_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/HeadingValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_HeadingValue(const HeadingValue_t& in, cam_msgs::HeadingValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_HeadingValue(const cam_msgs::HeadingValue& in, HeadingValue_t& out) {
    
  memset(&out, 0, sizeof(HeadingValue_t));
  toStruct_INTEGER(in.value, out);
}

}