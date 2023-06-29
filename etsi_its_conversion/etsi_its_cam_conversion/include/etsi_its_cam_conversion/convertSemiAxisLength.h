#pragma once

#include <etsi_its_cam_coding/SemiAxisLength.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/SemiAxisLength.h>


namespace etsi_its_cam_conversion {

void toRos_SemiAxisLength(const SemiAxisLength_t& in, etsi_its_cam_msgs::SemiAxisLength& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SemiAxisLength(const etsi_its_cam_msgs::SemiAxisLength& in, SemiAxisLength_t& out) {
    
  memset(&out, 0, sizeof(SemiAxisLength_t));
  toStruct_INTEGER(in.value, out);
}

}