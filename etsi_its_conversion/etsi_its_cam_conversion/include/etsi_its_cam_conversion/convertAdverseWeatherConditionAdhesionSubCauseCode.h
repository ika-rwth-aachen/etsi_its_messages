//// INTEGER AdverseWeatherConditionAdhesionSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/AdverseWeatherCondition-AdhesionSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/AdverseWeatherConditionAdhesionSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/adverse_weather_condition_adhesion_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_AdverseWeatherConditionAdhesionSubCauseCode(const AdverseWeatherCondition_AdhesionSubCauseCode_t& in, cam_msgs::AdverseWeatherConditionAdhesionSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AdverseWeatherConditionAdhesionSubCauseCode(const cam_msgs::AdverseWeatherConditionAdhesionSubCauseCode& in, AdverseWeatherCondition_AdhesionSubCauseCode_t& out) {
  memset(&out, 0, sizeof(AdverseWeatherCondition_AdhesionSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
