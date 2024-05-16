//// INTEGER HazardousLocationSurfaceConditionSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/HazardousLocation-SurfaceConditionSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/HazardousLocationSurfaceConditionSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/hazardous_location_surface_condition_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_HazardousLocationSurfaceConditionSubCauseCode(const HazardousLocation_SurfaceConditionSubCauseCode_t& in, denm_msgs::HazardousLocationSurfaceConditionSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HazardousLocationSurfaceConditionSubCauseCode(const denm_msgs::HazardousLocationSurfaceConditionSubCauseCode& in, HazardousLocation_SurfaceConditionSubCauseCode_t& out) {
  memset(&out, 0, sizeof(HazardousLocation_SurfaceConditionSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
