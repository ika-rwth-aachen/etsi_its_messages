//// SEQUENCE-OF ProtectedCommunicationZonesRSU


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/ProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_conversion/convertProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_conversion/convertProtectedCommunicationZone.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ProtectedCommunicationZonesRSU.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/protected_communication_zones_rsu.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ProtectedCommunicationZonesRSU(const ProtectedCommunicationZonesRSU_t& in, cam_msgs::ProtectedCommunicationZonesRSU& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cam_msgs::ProtectedCommunicationZone el;
    toRos_ProtectedCommunicationZone(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_ProtectedCommunicationZonesRSU(const cam_msgs::ProtectedCommunicationZonesRSU& in, ProtectedCommunicationZonesRSU_t& out) {
  memset(&out, 0, sizeof(ProtectedCommunicationZonesRSU_t));

  for (int i = 0; i < in.array.size(); ++i) {
    ProtectedCommunicationZone_t* el = (ProtectedCommunicationZone_t*) calloc(1, sizeof(ProtectedCommunicationZone_t));
    toStruct_ProtectedCommunicationZone(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
