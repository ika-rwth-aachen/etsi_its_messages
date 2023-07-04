#pragma once

#include <etsi_its_cam_coding/PosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>
#include <etsi_its_cam_msgs/PosConfidenceEllipse.h>


namespace etsi_its_cam_conversion {

void toRos_PosConfidenceEllipse(const PosConfidenceEllipse_t& in, etsi_its_cam_msgs::PosConfidenceEllipse& out) {

  toRos_SemiAxisLength(in.semiMajorConfidence, out.semiMajorConfidence);
  toRos_SemiAxisLength(in.semiMinorConfidence, out.semiMinorConfidence);
  toRos_HeadingValue(in.semiMajorOrientation, out.semiMajorOrientation);
}

void toStruct_PosConfidenceEllipse(const etsi_its_cam_msgs::PosConfidenceEllipse& in, PosConfidenceEllipse_t& out) {
    
  memset(&out, 0, sizeof(PosConfidenceEllipse_t));

  toStruct_SemiAxisLength(in.semiMajorConfidence, out.semiMajorConfidence);
  toStruct_SemiAxisLength(in.semiMinorConfidence, out.semiMinorConfidence);
  toStruct_HeadingValue(in.semiMajorOrientation, out.semiMajorOrientation);
}

}