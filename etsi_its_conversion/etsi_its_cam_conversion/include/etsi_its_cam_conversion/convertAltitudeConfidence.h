#pragma once

#include <etsi_its_cam_coding/AltitudeConfidence.h>
#include <etsi_its_cam_msgs/AltitudeConfidence.h>


namespace etsi_its_cam_conversion {

void toRos_AltitudeConfidence(const AltitudeConfidence_t& in, etsi_its_cam_msgs::AltitudeConfidence& out) {

  out.value = in;
}

void toStruct_AltitudeConfidence(const etsi_its_cam_msgs::AltitudeConfidence& in, AltitudeConfidence_t& out) {
    
  memset(&out, 0, sizeof(AltitudeConfidence_t));
  out = in.value;
}

}