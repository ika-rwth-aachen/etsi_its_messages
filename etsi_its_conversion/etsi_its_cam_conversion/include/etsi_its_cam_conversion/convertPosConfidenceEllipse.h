#pragma once

#include <etsi_its_cam_coding/PosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/pos_confidence_ellipse.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/PosConfidenceEllipse.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_PosConfidenceEllipse(const PosConfidenceEllipse_t& in, cam_msgs::PosConfidenceEllipse& out) {

  toRos_SemiAxisLength(in.semiMajorConfidence, out.semi_major_confidence);
  toRos_SemiAxisLength(in.semiMinorConfidence, out.semi_minor_confidence);
  toRos_HeadingValue(in.semiMajorOrientation, out.semi_major_orientation);
}

void toStruct_PosConfidenceEllipse(const cam_msgs::PosConfidenceEllipse& in, PosConfidenceEllipse_t& out) {
    
  memset(&out, 0, sizeof(PosConfidenceEllipse_t));

  toStruct_SemiAxisLength(in.semi_major_confidence, out.semiMajorConfidence);
  toStruct_SemiAxisLength(in.semi_minor_confidence, out.semiMinorConfidence);
  toStruct_HeadingValue(in.semi_major_orientation, out.semiMajorOrientation);
}

}