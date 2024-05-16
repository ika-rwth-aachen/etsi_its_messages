//// INTEGER AdverseWeatherConditionVisibilitySubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/AdverseWeatherCondition-VisibilitySubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/AdverseWeatherConditionVisibilitySubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/adverse_weather_condition_visibility_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_AdverseWeatherConditionVisibilitySubCauseCode(const AdverseWeatherCondition_VisibilitySubCauseCode_t& in, cam_msgs::AdverseWeatherConditionVisibilitySubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AdverseWeatherConditionVisibilitySubCauseCode(const cam_msgs::AdverseWeatherConditionVisibilitySubCauseCode& in, AdverseWeatherCondition_VisibilitySubCauseCode_t& out) {
  memset(&out, 0, sizeof(AdverseWeatherCondition_VisibilitySubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
