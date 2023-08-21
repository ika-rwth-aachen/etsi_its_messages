#pragma once

#include <etsi_its_cam_coding/PosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>
#include <etsi_its_cam_msgs/PosConfidenceEllipse.h>


namespace etsi_its_cam_conversion {

void toRos_PosConfidenceEllipse(const PosConfidenceEllipse_t& in, etsi_its_cam_msgs::PosConfidenceEllipse& out) {

  toRos_SemiAxisLength(in.semi_major_confidence, out.semi_major_confidence);
  toRos_SemiAxisLength(in.semi_minor_confidence, out.semi_minor_confidence);
  toRos_HeadingValue(in.semi_major_orientation, out.semi_major_orientation);
}

void toStruct_PosConfidenceEllipse(const etsi_its_cam_msgs::PosConfidenceEllipse& in, PosConfidenceEllipse_t& out) {
    
  memset(&out, 0, sizeof(PosConfidenceEllipse_t));

  toStruct_SemiAxisLength(in.semi_major_confidence, out.semi_major_confidence);
  toStruct_SemiAxisLength(in.semi_minor_confidence, out.semi_minor_confidence);
  toStruct_HeadingValue(in.semi_major_orientation, out.semi_major_orientation);
}

}