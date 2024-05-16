//// INTEGER HazardousLocationDangerousCurveSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/HazardousLocation-DangerousCurveSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/HazardousLocationDangerousCurveSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/hazardous_location_dangerous_curve_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_HazardousLocationDangerousCurveSubCauseCode(const HazardousLocation_DangerousCurveSubCauseCode_t& in, cam_msgs::HazardousLocationDangerousCurveSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HazardousLocationDangerousCurveSubCauseCode(const cam_msgs::HazardousLocationDangerousCurveSubCauseCode& in, HazardousLocation_DangerousCurveSubCauseCode_t& out) {
  memset(&out, 0, sizeof(HazardousLocation_DangerousCurveSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
