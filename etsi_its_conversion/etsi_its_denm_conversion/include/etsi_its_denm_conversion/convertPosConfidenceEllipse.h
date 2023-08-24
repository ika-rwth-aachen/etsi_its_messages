#pragma once

#include <etsi_its_denm_coding/PosConfidenceEllipse.h>
#include <etsi_its_denm_conversion/convertSemiAxisLength.h>
#include <etsi_its_denm_conversion/convertSemiAxisLength.h>
#include <etsi_its_denm_conversion/convertHeadingValue.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/pos_confidence_ellipse.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PosConfidenceEllipse.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PosConfidenceEllipse(const PosConfidenceEllipse_t& in, denm_msgs::PosConfidenceEllipse& out) {

  toRos_SemiAxisLength(in.semiMajorConfidence, out.semi_major_confidence);
  toRos_SemiAxisLength(in.semiMinorConfidence, out.semi_minor_confidence);
  toRos_HeadingValue(in.semiMajorOrientation, out.semi_major_orientation);
}

void toStruct_PosConfidenceEllipse(const denm_msgs::PosConfidenceEllipse& in, PosConfidenceEllipse_t& out) {
    
  memset(&out, 0, sizeof(PosConfidenceEllipse_t));

  toStruct_SemiAxisLength(in.semi_major_confidence, out.semiMajorConfidence);
  toStruct_SemiAxisLength(in.semi_minor_confidence, out.semiMinorConfidence);
  toStruct_HeadingValue(in.semi_major_orientation, out.semiMajorOrientation);
}

}