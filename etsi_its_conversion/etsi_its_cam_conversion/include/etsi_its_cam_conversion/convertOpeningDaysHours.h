//// UTF8String OpeningDaysHours


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/OpeningDaysHours.h>
#include <etsi_its_cam_coding/UTF8String.h>
#include <etsi_its_primitives_conversion/convertUTF8String.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/OpeningDaysHours.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/opening_days_hours.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_OpeningDaysHours(const OpeningDaysHours_t& in, cam_msgs::OpeningDaysHours& out) {
  etsi_its_primitives_conversion::toRos_UTF8String(in, out.value);
}

void toStruct_OpeningDaysHours(const cam_msgs::OpeningDaysHours& in, OpeningDaysHours_t& out) {
  memset(&out, 0, sizeof(OpeningDaysHours_t));

  etsi_its_primitives_conversion::toStruct_UTF8String(in.value, out);
}

}
