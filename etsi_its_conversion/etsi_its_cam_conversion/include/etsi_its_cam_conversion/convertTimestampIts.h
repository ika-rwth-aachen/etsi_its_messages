//// INTEGER TimestampIts


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/TimestampIts.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/TimestampIts.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/timestamp_its.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_TimestampIts(const TimestampIts_t& in, cam_msgs::TimestampIts& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_TimestampIts(const cam_msgs::TimestampIts& in, TimestampIts_t& out) {
  memset(&out, 0, sizeof(TimestampIts_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
