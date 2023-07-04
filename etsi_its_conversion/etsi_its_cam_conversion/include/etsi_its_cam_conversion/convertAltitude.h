#pragma once

#include <etsi_its_cam_coding/Altitude.h>
#include <etsi_its_cam_conversion/convertAltitudeValue.h>
#include <etsi_its_cam_conversion/convertAltitudeConfidence.h>
#include <etsi_its_cam_msgs/Altitude.h>


namespace etsi_its_cam_conversion {

void toRos_Altitude(const Altitude_t& in, etsi_its_cam_msgs::Altitude& out) {

  toRos_AltitudeValue(in.altitudeValue, out.altitudeValue);
  toRos_AltitudeConfidence(in.altitudeConfidence, out.altitudeConfidence);
}

void toStruct_Altitude(const etsi_its_cam_msgs::Altitude& in, Altitude_t& out) {
    
  memset(&out, 0, sizeof(Altitude_t));

  toStruct_AltitudeValue(in.altitudeValue, out.altitudeValue);
  toStruct_AltitudeConfidence(in.altitudeConfidence, out.altitudeConfidence);
}

}