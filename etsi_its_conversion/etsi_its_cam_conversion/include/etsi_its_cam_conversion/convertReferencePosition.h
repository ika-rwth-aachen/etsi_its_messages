#pragma once

#include <etsi_its_cam_coding/ReferencePosition.h>
#include <etsi_its_cam_msgs/ReferencePosition.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertPosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertAltitude.h>

namespace etsi_its_cam_conversion {
  
void toRos_ReferencePosition(const ReferencePosition_t& in, etsi_its_cam_msgs::ReferencePosition& out) {
  toRos_Latitude(in.latitude, out.latitude);
  toRos_Longitude(in.longitude, out.longitude);
  toRos_PosConfidenceEllipse(in.positionConfidenceEllipse, out.positionConfidenceEllipse);
  toRos_Altitude(in.altitude, out.altitude);
}

void toStruct_ReferencePosition(const etsi_its_cam_msgs::ReferencePosition& in, ReferencePosition_t& out) {
  memset(&out, 0, sizeof(ReferencePosition_t));
  toStruct_Latitude(in.latitude, out.latitude);
  toStruct_Longitude(in.longitude, out.longitude);
  toStruct_PosConfidenceEllipse(in.positionConfidenceEllipse, out.positionConfidenceEllipse);
  toStruct_Altitude(in.altitude, out.altitude);
}

}