//// INTEGER CollisionRiskSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/CollisionRiskSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/CollisionRiskSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/collision_risk_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_CollisionRiskSubCauseCode(const CollisionRiskSubCauseCode_t& in, denm_msgs::CollisionRiskSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_CollisionRiskSubCauseCode(const denm_msgs::CollisionRiskSubCauseCode& in, CollisionRiskSubCauseCode_t& out) {
  memset(&out, 0, sizeof(CollisionRiskSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
