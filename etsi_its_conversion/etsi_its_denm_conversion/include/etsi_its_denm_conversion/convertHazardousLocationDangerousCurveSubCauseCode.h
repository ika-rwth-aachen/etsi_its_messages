//// INTEGER HazardousLocationDangerousCurveSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/HazardousLocation-DangerousCurveSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/HazardousLocationDangerousCurveSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/hazardous_location_dangerous_curve_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_HazardousLocationDangerousCurveSubCauseCode(const HazardousLocation_DangerousCurveSubCauseCode_t& in, denm_msgs::HazardousLocationDangerousCurveSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HazardousLocationDangerousCurveSubCauseCode(const denm_msgs::HazardousLocationDangerousCurveSubCauseCode& in, HazardousLocation_DangerousCurveSubCauseCode_t& out) {
  memset(&out, 0, sizeof(HazardousLocation_DangerousCurveSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
