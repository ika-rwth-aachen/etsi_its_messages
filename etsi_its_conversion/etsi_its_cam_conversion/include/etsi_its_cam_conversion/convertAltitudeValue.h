#pragma once

#include <etsi_its_cam_coding/AltitudeValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/AltitudeValue.h>


namespace etsi_its_cam_conversion {

void toRos_AltitudeValue(const AltitudeValue_t& in, etsi_its_cam_msgs::AltitudeValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_AltitudeValue(const etsi_its_cam_msgs::AltitudeValue& in, AltitudeValue_t& out) {
    
  memset(&out, 0, sizeof(AltitudeValue_t));
  toStruct_INTEGER(in.value, out);
}

}