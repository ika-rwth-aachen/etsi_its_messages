//// INTEGER CollisionRiskSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/CollisionRiskSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CollisionRiskSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/collision_risk_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CollisionRiskSubCauseCode(const CollisionRiskSubCauseCode_t& in, cam_msgs::CollisionRiskSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_CollisionRiskSubCauseCode(const cam_msgs::CollisionRiskSubCauseCode& in, CollisionRiskSubCauseCode_t& out) {
  memset(&out, 0, sizeof(CollisionRiskSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
