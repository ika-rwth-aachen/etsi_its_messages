#pragma once

#include <etsi_its_cam_coding/DeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertDeltaLatitude.h>
#include <etsi_its_cam_conversion/convertDeltaLongitude.h>
#include <etsi_its_cam_conversion/convertDeltaAltitude.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/DeltaReferencePosition.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/delta_reference_position.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_DeltaReferencePosition(const DeltaReferencePosition_t& in, cam_msgs::DeltaReferencePosition& out) {

  toRos_DeltaLatitude(in.deltaLatitude, out.delta_latitude);
  toRos_DeltaLongitude(in.deltaLongitude, out.delta_longitude);
  toRos_DeltaAltitude(in.deltaAltitude, out.delta_altitude);
}

void toStruct_DeltaReferencePosition(const cam_msgs::DeltaReferencePosition& in, DeltaReferencePosition_t& out) {

  memset(&out, 0, sizeof(DeltaReferencePosition_t));

  toStruct_DeltaLatitude(in.delta_latitude, out.deltaLatitude);
  toStruct_DeltaLongitude(in.delta_longitude, out.deltaLongitude);
  toStruct_DeltaAltitude(in.delta_altitude, out.deltaAltitude);
}

}