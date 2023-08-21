#pragma once

#include <etsi_its_cam_coding/Heading.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>
#include <etsi_its_cam_conversion/convertHeadingConfidence.h>
#include <etsi_its_cam_msgs/Heading.h>


namespace etsi_its_cam_conversion {

void toRos_Heading(const Heading_t& in, etsi_its_cam_msgs::Heading& out) {

  toRos_HeadingValue(in.headingValue, out.heading_value);
  toRos_HeadingConfidence(in.headingConfidence, out.heading_confidence);
}

void toStruct_Heading(const etsi_its_cam_msgs::Heading& in, Heading_t& out) {
    
  memset(&out, 0, sizeof(Heading_t));

  toStruct_HeadingValue(in.heading_value, out.headingValue);
  toStruct_HeadingConfidence(in.heading_confidence, out.headingConfidence);
}

}