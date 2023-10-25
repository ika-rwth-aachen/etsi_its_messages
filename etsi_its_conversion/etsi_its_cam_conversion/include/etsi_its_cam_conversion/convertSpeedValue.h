#pragma once

#include <etsi_its_cam_coding/SpeedValue.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SpeedValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/speed_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SpeedValue(const SpeedValue_t& in, cam_msgs::SpeedValue& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SpeedValue(const cam_msgs::SpeedValue& in, SpeedValue_t& out) {

  memset(&out, 0, sizeof(SpeedValue_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}