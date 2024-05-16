//// INTEGER VehicleWidth


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/VehicleWidth.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/VehicleWidth.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vehicle_width.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VehicleWidth(const VehicleWidth_t& in, denm_msgs::VehicleWidth& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_VehicleWidth(const denm_msgs::VehicleWidth& in, VehicleWidth_t& out) {
  memset(&out, 0, sizeof(VehicleWidth_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
