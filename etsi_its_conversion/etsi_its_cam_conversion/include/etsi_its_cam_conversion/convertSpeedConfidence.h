#pragma once

#include <etsi_its_cam_coding/SpeedConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/SpeedConfidence.h>


namespace etsi_its_cam_conversion {

void toRos_SpeedConfidence(const SpeedConfidence_t& in, etsi_its_cam_msgs::SpeedConfidence& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SpeedConfidence(const etsi_its_cam_msgs::SpeedConfidence& in, SpeedConfidence_t& out) {
    
  memset(&out, 0, sizeof(SpeedConfidence_t));
  toStruct_INTEGER(in.value, out);
}

}