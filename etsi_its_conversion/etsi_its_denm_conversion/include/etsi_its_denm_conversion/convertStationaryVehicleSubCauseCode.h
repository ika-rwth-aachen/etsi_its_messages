//// INTEGER StationaryVehicleSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/StationaryVehicleSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/StationaryVehicleSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/stationary_vehicle_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_StationaryVehicleSubCauseCode(const StationaryVehicleSubCauseCode_t& in, denm_msgs::StationaryVehicleSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_StationaryVehicleSubCauseCode(const denm_msgs::StationaryVehicleSubCauseCode& in, StationaryVehicleSubCauseCode_t& out) {
  memset(&out, 0, sizeof(StationaryVehicleSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
