#pragma once

#include <etsi_its_cam_coding/Altitude.h>
#include <etsi_its_cam_conversion/convertAltitudeValue.h>
#include <etsi_its_cam_conversion/convertAltitudeConfidence.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/altitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/Altitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_Altitude(const Altitude_t& in, cam_msgs::Altitude& out) {

  toRos_AltitudeValue(in.altitudeValue, out.altitude_value);
  toRos_AltitudeConfidence(in.altitudeConfidence, out.altitude_confidence);
}

void toStruct_Altitude(const cam_msgs::Altitude& in, Altitude_t& out) {
    
  memset(&out, 0, sizeof(Altitude_t));

  toStruct_AltitudeValue(in.altitude_value, out.altitudeValue);
  toStruct_AltitudeConfidence(in.altitude_confidence, out.altitudeConfidence);
}

}