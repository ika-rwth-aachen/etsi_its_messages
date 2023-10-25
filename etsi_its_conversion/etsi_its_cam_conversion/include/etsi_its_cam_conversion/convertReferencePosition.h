#pragma once

#include <etsi_its_cam_coding/ReferencePosition.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertPosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertAltitude.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ReferencePosition.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/reference_position.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ReferencePosition(const ReferencePosition_t& in, cam_msgs::ReferencePosition& out) {

  toRos_Latitude(in.latitude, out.latitude);
  toRos_Longitude(in.longitude, out.longitude);
  toRos_PosConfidenceEllipse(in.positionConfidenceEllipse, out.position_confidence_ellipse);
  toRos_Altitude(in.altitude, out.altitude);
}

void toStruct_ReferencePosition(const cam_msgs::ReferencePosition& in, ReferencePosition_t& out) {

  memset(&out, 0, sizeof(ReferencePosition_t));

  toStruct_Latitude(in.latitude, out.latitude);
  toStruct_Longitude(in.longitude, out.longitude);
  toStruct_PosConfidenceEllipse(in.position_confidence_ellipse, out.positionConfidenceEllipse);
  toStruct_Altitude(in.altitude, out.altitude);
}

}