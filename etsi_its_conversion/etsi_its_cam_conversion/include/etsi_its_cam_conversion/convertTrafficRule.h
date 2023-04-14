#pragma once

#include <etsi_its_cam_coding/TrafficRule.h>
#include <etsi_its_cam_msgs/TrafficRule.h>

namespace etsi_its_cam_conversion {
  
void convert_TrafficRuletoRos(const TrafficRule_t& _TrafficRule_in, etsi_its_cam_msgs::TrafficRule& _TrafficRule_out) {
  _TrafficRule_out.value = _TrafficRule_in;
}

void convert_TrafficRuletoC(const etsi_its_cam_msgs::TrafficRule& _TrafficRule_in, TrafficRule_t& _TrafficRule_out) {
  memset(&_TrafficRule_out, 0, sizeof(TrafficRule_t));
  _TrafficRule_out = _TrafficRule_in.value;
}

}