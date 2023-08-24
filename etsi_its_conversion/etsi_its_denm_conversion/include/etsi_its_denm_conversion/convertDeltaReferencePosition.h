#pragma once

#include <etsi_its_denm_coding/DeltaReferencePosition.h>
#include <etsi_its_denm_conversion/convertDeltaLatitude.h>
#include <etsi_its_denm_conversion/convertDeltaLongitude.h>
#include <etsi_its_denm_conversion/convertDeltaAltitude.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/delta_reference_position.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/DeltaReferencePosition.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_DeltaReferencePosition(const DeltaReferencePosition_t& in, denm_msgs::DeltaReferencePosition& out) {

  toRos_DeltaLatitude(in.deltaLatitude, out.delta_latitude);
  toRos_DeltaLongitude(in.deltaLongitude, out.delta_longitude);
  toRos_DeltaAltitude(in.deltaAltitude, out.delta_altitude);
}

void toStruct_DeltaReferencePosition(const denm_msgs::DeltaReferencePosition& in, DeltaReferencePosition_t& out) {
    
  memset(&out, 0, sizeof(DeltaReferencePosition_t));

  toStruct_DeltaLatitude(in.delta_latitude, out.deltaLatitude);
  toStruct_DeltaLongitude(in.delta_longitude, out.deltaLongitude);
  toStruct_DeltaAltitude(in.delta_altitude, out.deltaAltitude);
}

}