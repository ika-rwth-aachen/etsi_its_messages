//// INTEGER SlowVehicleSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/SlowVehicleSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SlowVehicleSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/slow_vehicle_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SlowVehicleSubCauseCode(const SlowVehicleSubCauseCode_t& in, denm_msgs::SlowVehicleSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SlowVehicleSubCauseCode(const denm_msgs::SlowVehicleSubCauseCode& in, SlowVehicleSubCauseCode_t& out) {
  memset(&out, 0, sizeof(SlowVehicleSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
