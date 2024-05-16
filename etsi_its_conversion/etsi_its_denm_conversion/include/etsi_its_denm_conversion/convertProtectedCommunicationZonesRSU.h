//// SEQUENCE-OF ProtectedCommunicationZonesRSU


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/ProtectedCommunicationZonesRSU.h>
#include <etsi_its_denm_conversion/convertProtectedCommunicationZonesRSU.h>
#include <etsi_its_denm_conversion/convertProtectedCommunicationZone.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/ProtectedCommunicationZonesRSU.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/protected_communication_zones_rsu.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_ProtectedCommunicationZonesRSU(const ProtectedCommunicationZonesRSU_t& in, denm_msgs::ProtectedCommunicationZonesRSU& out) {
  for (int i = 0; i < in.list.count; ++i) {
    denm_msgs::ProtectedCommunicationZone el;
    toRos_ProtectedCommunicationZone(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_ProtectedCommunicationZonesRSU(const denm_msgs::ProtectedCommunicationZonesRSU& in, ProtectedCommunicationZonesRSU_t& out) {
  memset(&out, 0, sizeof(ProtectedCommunicationZonesRSU_t));

  for (int i = 0; i < in.array.size(); ++i) {
    ProtectedCommunicationZone_t* el = (ProtectedCommunicationZone_t*) calloc(1, sizeof(ProtectedCommunicationZone_t));
    toStruct_ProtectedCommunicationZone(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
