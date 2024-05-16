//// INTEGER WrongWayDrivingSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/WrongWayDrivingSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/WrongWayDrivingSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/wrong_way_driving_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_WrongWayDrivingSubCauseCode(const WrongWayDrivingSubCauseCode_t& in, denm_msgs::WrongWayDrivingSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_WrongWayDrivingSubCauseCode(const denm_msgs::WrongWayDrivingSubCauseCode& in, WrongWayDrivingSubCauseCode_t& out) {
  memset(&out, 0, sizeof(WrongWayDrivingSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
