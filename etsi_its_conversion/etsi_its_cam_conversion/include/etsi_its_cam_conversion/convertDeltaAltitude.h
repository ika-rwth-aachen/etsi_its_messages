#pragma once

#include <etsi_its_cam_coding/DeltaAltitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/DeltaAltitude.h>


namespace etsi_its_cam_conversion {

void toRos_DeltaAltitude(const DeltaAltitude_t& in, etsi_its_cam_msgs::DeltaAltitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_DeltaAltitude(const etsi_its_cam_msgs::DeltaAltitude& in, DeltaAltitude_t& out) {
    
  memset(&out, 0, sizeof(DeltaAltitude_t));
  toStruct_INTEGER(in.value, out);
}

}