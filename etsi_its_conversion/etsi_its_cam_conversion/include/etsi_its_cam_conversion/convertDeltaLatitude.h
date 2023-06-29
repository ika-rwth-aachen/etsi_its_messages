#pragma once

#include <etsi_its_cam_coding/DeltaLatitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/DeltaLatitude.h>


namespace etsi_its_cam_conversion {

void toRos_DeltaLatitude(const DeltaLatitude_t& in, etsi_its_cam_msgs::DeltaLatitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLatitude(const etsi_its_cam_msgs::DeltaLatitude& in, DeltaLatitude_t& out) {
    
  memset(&out, 0, sizeof(DeltaLatitude_t));
  toStruct_INTEGER(in.value, out);
}

}