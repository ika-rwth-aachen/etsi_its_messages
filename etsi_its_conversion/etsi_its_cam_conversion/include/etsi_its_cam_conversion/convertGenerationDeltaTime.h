#pragma once

#include <etsi_its_cam_coding/GenerationDeltaTime.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/GenerationDeltaTime.h>


namespace etsi_its_cam_conversion {

void toRos_GenerationDeltaTime(const GenerationDeltaTime_t& in, etsi_its_cam_msgs::GenerationDeltaTime& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_GenerationDeltaTime(const etsi_its_cam_msgs::GenerationDeltaTime& in, GenerationDeltaTime_t& out) {
    
  memset(&out, 0, sizeof(GenerationDeltaTime_t));
  toStruct_INTEGER(in.value, out);
}

}