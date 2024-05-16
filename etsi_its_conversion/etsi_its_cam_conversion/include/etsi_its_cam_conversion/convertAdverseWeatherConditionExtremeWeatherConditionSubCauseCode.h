//// INTEGER AdverseWeatherConditionExtremeWeatherConditionSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/AdverseWeatherCondition-ExtremeWeatherConditionSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/AdverseWeatherConditionExtremeWeatherConditionSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/adverse_weather_condition_extreme_weather_condition_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_AdverseWeatherConditionExtremeWeatherConditionSubCauseCode(const AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_t& in, cam_msgs::AdverseWeatherConditionExtremeWeatherConditionSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AdverseWeatherConditionExtremeWeatherConditionSubCauseCode(const cam_msgs::AdverseWeatherConditionExtremeWeatherConditionSubCauseCode& in, AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_t& out) {
  memset(&out, 0, sizeof(AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
