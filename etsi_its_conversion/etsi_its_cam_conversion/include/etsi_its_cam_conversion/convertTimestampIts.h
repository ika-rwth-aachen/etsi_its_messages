#pragma once

#include <etsi_its_cam_coding/TimestampIts.h>
#include <etsi_its_cam_msgs/TimestampIts.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_TimestampIts(const TimestampIts_t& in, etsi_its_cam_msgs::TimestampIts& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_TimestampIts(const etsi_its_cam_msgs::TimestampIts& in, TimestampIts_t& out) {
  memset(&out, 0, sizeof(TimestampIts_t));
  toStruct_INTEGER(in.value, out);

}

}