#pragma once

#include <etsi_its_cam_coding/DeltaLongitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/DeltaLongitude.h>


namespace etsi_its_cam_conversion {

void toRos_DeltaLongitude(const DeltaLongitude_t& in, etsi_its_cam_msgs::DeltaLongitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLongitude(const etsi_its_cam_msgs::DeltaLongitude& in, DeltaLongitude_t& out) {
    
  memset(&out, 0, sizeof(DeltaLongitude_t));
  toStruct_INTEGER(in.value, out);
}

}