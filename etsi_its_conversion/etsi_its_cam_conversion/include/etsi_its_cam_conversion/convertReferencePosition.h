#pragma once

#include <etsi_its_cam_coding/ReferencePosition.h>
#include <etsi_its_cam_msgs/ReferencePosition.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertPosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertAltitude.h>

namespace etsi_its_cam_conversion {
  
void convert_ReferencePositiontoRos(const ReferencePosition_t& _ReferencePosition_in, etsi_its_cam_msgs::ReferencePosition& _ReferencePosition_out) {
  convert_LatitudetoRos(_ReferencePosition_in.latitude, _ReferencePosition_out.latitude);
  convert_LongitudetoRos(_ReferencePosition_in.longitude, _ReferencePosition_out.longitude);
  convert_PosConfidenceEllipsetoRos(_ReferencePosition_in.positionConfidenceEllipse, _ReferencePosition_out.positionConfidenceEllipse);
  convert_AltitudetoRos(_ReferencePosition_in.altitude, _ReferencePosition_out.altitude);
}

void convert_ReferencePositiontoC(const etsi_its_cam_msgs::ReferencePosition& _ReferencePosition_in, ReferencePosition_t& _ReferencePosition_out) {
  memset(&_ReferencePosition_out, 0, sizeof(ReferencePosition_t));
  convert_LatitudetoC(_ReferencePosition_in.latitude, _ReferencePosition_out.latitude);
  convert_LongitudetoC(_ReferencePosition_in.longitude, _ReferencePosition_out.longitude);
  convert_PosConfidenceEllipsetoC(_ReferencePosition_in.positionConfidenceEllipse, _ReferencePosition_out.positionConfidenceEllipse);
  convert_AltitudetoC(_ReferencePosition_in.altitude, _ReferencePosition_out.altitude);
}

}