//// INTEGER AdverseWeatherConditionPrecipitationSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/AdverseWeatherCondition-PrecipitationSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/AdverseWeatherConditionPrecipitationSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/adverse_weather_condition_precipitation_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_AdverseWeatherConditionPrecipitationSubCauseCode(const AdverseWeatherCondition_PrecipitationSubCauseCode_t& in, denm_msgs::AdverseWeatherConditionPrecipitationSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_AdverseWeatherConditionPrecipitationSubCauseCode(const denm_msgs::AdverseWeatherConditionPrecipitationSubCauseCode& in, AdverseWeatherCondition_PrecipitationSubCauseCode_t& out) {
  memset(&out, 0, sizeof(AdverseWeatherCondition_PrecipitationSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
