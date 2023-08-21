#pragma once

#include <etsi_its_cam_coding/VehicleLengthConfidenceIndication.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/vehicle_length_confidence_indication.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/VehicleLengthConfidenceIndication.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_VehicleLengthConfidenceIndication(const VehicleLengthConfidenceIndication_t& in, cam_msgs::VehicleLengthConfidenceIndication& out) {

  out.value = in;
}

void toStruct_VehicleLengthConfidenceIndication(const cam_msgs::VehicleLengthConfidenceIndication& in, VehicleLengthConfidenceIndication_t& out) {
    
  memset(&out, 0, sizeof(VehicleLengthConfidenceIndication_t));
  out = in.value;
}

}