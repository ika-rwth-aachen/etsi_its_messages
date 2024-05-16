//// INTEGER GenerationDeltaTime


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/GenerationDeltaTime.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/GenerationDeltaTime.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/generation_delta_time.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_GenerationDeltaTime(const GenerationDeltaTime_t& in, cam_msgs::GenerationDeltaTime& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_GenerationDeltaTime(const cam_msgs::GenerationDeltaTime& in, GenerationDeltaTime_t& out) {
  memset(&out, 0, sizeof(GenerationDeltaTime_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
