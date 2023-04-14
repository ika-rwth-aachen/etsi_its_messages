#pragma once

#include <etsi_its_cam_coding/DeltaReferencePosition.h>
#include <etsi_its_cam_msgs/DeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertDeltaLatitude.h>
#include <etsi_its_cam_conversion/convertDeltaLongitude.h>
#include <etsi_its_cam_conversion/convertDeltaAltitude.h>

namespace etsi_its_cam_conversion {
  
void toRos_DeltaReferencePosition(const DeltaReferencePosition_t& in, etsi_its_cam_msgs::DeltaReferencePosition& out) {
  toRos_DeltaLatitude(in.deltaLatitude, out.deltaLatitude);
  toRos_DeltaLongitude(in.deltaLongitude, out.deltaLongitude);
  toRos_DeltaAltitude(in.deltaAltitude, out.deltaAltitude);
}

void toStruct_DeltaReferencePosition(const etsi_its_cam_msgs::DeltaReferencePosition& in, DeltaReferencePosition_t& out) {
  memset(&out, 0, sizeof(DeltaReferencePosition_t));
  toStruct_DeltaLatitude(in.deltaLatitude, out.deltaLatitude);
  toStruct_DeltaLongitude(in.deltaLongitude, out.deltaLongitude);
  toStruct_DeltaAltitude(in.deltaAltitude, out.deltaAltitude);
}

}