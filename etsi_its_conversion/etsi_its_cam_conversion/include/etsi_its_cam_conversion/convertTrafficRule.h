#pragma once

#include <etsi_its_cam_coding/TrafficRule.h>
#include <etsi_its_cam_msgs/TrafficRule.h>

namespace etsi_its_cam_conversion {
  
void toRos_TrafficRule(const TrafficRule_t& in, etsi_its_cam_msgs::TrafficRule& out) {
  out.value = in;
}

void toStruct_TrafficRule(const etsi_its_cam_msgs::TrafficRule& in, TrafficRule_t& out) {
  memset(&out, 0, sizeof(TrafficRule_t));
  out = in.value;
}

}