/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

// --- Auto-generated by asn1ToConversionHeader.py -----------------------------

#pragma once

#include <stdexcept>

#include <etsi_its_cpm_coding/cpm_ProtectedCommunicationZonesRSU.h>
#include <etsi_its_cpm_conversion/convertProtectedCommunicationZone.h>
#include <etsi_its_cpm_conversion/convertProtectedCommunicationZonesRSU.h>
#ifdef ROS1
#include <etsi_its_cpm_msgs/ProtectedCommunicationZonesRSU.h>
namespace cpm_msgs = etsi_its_cpm_msgs;
#else
#include <etsi_its_cpm_msgs/msg/protected_communication_zones_rsu.hpp>
namespace cpm_msgs = etsi_its_cpm_msgs::msg;
#endif


namespace etsi_its_cpm_conversion {

void toRos_ProtectedCommunicationZonesRSU(const cpm_ProtectedCommunicationZonesRSU_t& in, cpm_msgs::ProtectedCommunicationZonesRSU& out) {
  for (int i = 0; i < in.list.count; ++i) {
    cpm_msgs::ProtectedCommunicationZone el;
    toRos_ProtectedCommunicationZone(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_ProtectedCommunicationZonesRSU(const cpm_msgs::ProtectedCommunicationZonesRSU& in, cpm_ProtectedCommunicationZonesRSU_t& out) {
  memset(&out, 0, sizeof(cpm_ProtectedCommunicationZonesRSU_t));

  for (int i = 0; i < in.array.size(); ++i) {
    cpm_ProtectedCommunicationZone_t* el = (cpm_ProtectedCommunicationZone_t*) calloc(1, sizeof(cpm_ProtectedCommunicationZone_t));
    toStruct_ProtectedCommunicationZone(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
