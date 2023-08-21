#pragma once

#include <etsi_its_cam_coding/Altitude.h>
#include <etsi_its_cam_conversion/convertAltitudeValue.h>
#include <etsi_its_cam_conversion/convertAltitudeConfidence.h>
#include <etsi_its_cam_msgs/Altitude.h>


namespace etsi_its_cam_conversion {

void toRos_Altitude(const Altitude_t& in, etsi_its_cam_msgs::Altitude& out) {

  toRos_AltitudeValue(in.altitude_value, out.altitude_value);
  toRos_AltitudeConfidence(in.altitude_confidence, out.altitude_confidence);
}

void toStruct_Altitude(const etsi_its_cam_msgs::Altitude& in, Altitude_t& out) {
    
  memset(&out, 0, sizeof(Altitude_t));

  toStruct_AltitudeValue(in.altitude_value, out.altitude_value);
  toStruct_AltitudeConfidence(in.altitude_confidence, out.altitude_confidence);
}

}