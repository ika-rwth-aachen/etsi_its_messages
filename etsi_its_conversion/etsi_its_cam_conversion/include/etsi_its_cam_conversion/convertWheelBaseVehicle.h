//// INTEGER WheelBaseVehicle


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/WheelBaseVehicle.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/WheelBaseVehicle.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/wheel_base_vehicle.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_WheelBaseVehicle(const WheelBaseVehicle_t& in, cam_msgs::WheelBaseVehicle& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_WheelBaseVehicle(const cam_msgs::WheelBaseVehicle& in, WheelBaseVehicle_t& out) {
  memset(&out, 0, sizeof(WheelBaseVehicle_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
