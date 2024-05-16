//// INTEGER HazardousLocationAnimalOnTheRoadSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/HazardousLocation-AnimalOnTheRoadSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/HazardousLocationAnimalOnTheRoadSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/hazardous_location_animal_on_the_road_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_HazardousLocationAnimalOnTheRoadSubCauseCode(const HazardousLocation_AnimalOnTheRoadSubCauseCode_t& in, denm_msgs::HazardousLocationAnimalOnTheRoadSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HazardousLocationAnimalOnTheRoadSubCauseCode(const denm_msgs::HazardousLocationAnimalOnTheRoadSubCauseCode& in, HazardousLocation_AnimalOnTheRoadSubCauseCode_t& out) {
  memset(&out, 0, sizeof(HazardousLocation_AnimalOnTheRoadSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
