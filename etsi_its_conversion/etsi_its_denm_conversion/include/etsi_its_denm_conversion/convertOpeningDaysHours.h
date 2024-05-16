//// UTF8String OpeningDaysHours


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/OpeningDaysHours.h>
#include <etsi_its_denm_coding/UTF8String.h>
#include <etsi_its_primitives_conversion/convertUTF8String.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/OpeningDaysHours.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/opening_days_hours.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_OpeningDaysHours(const OpeningDaysHours_t& in, denm_msgs::OpeningDaysHours& out) {
  etsi_its_primitives_conversion::toRos_UTF8String(in, out.value);
}

void toStruct_OpeningDaysHours(const denm_msgs::OpeningDaysHours& in, OpeningDaysHours_t& out) {
  memset(&out, 0, sizeof(OpeningDaysHours_t));

  etsi_its_primitives_conversion::toStruct_UTF8String(in.value, out);
}

}
