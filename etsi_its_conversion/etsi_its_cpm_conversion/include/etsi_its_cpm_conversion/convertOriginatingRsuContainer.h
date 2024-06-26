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

#include <etsi_its_cpm_coding/cpm_OriginatingRsuContainer.h>
#include <etsi_its_cpm_conversion/convertMapReference.h>
#ifdef ROS1
#include <etsi_its_cpm_msgs/OriginatingRsuContainer.h>
namespace cpm_msgs = etsi_its_cpm_msgs;
#else
#include <etsi_its_cpm_msgs/msg/originating_rsu_container.hpp>
namespace cpm_msgs = etsi_its_cpm_msgs::msg;
#endif


namespace etsi_its_cpm_conversion {

void toRos_OriginatingRsuContainer(const cpm_OriginatingRsuContainer_t& in, cpm_msgs::OriginatingRsuContainer& out) {
  if (in.mapReference) {
    toRos_MapReference(*in.mapReference, out.map_reference);
    out.map_reference_is_present = true;
  }
}

void toStruct_OriginatingRsuContainer(const cpm_msgs::OriginatingRsuContainer& in, cpm_OriginatingRsuContainer_t& out) {
  memset(&out, 0, sizeof(cpm_OriginatingRsuContainer_t));

  if (in.map_reference_is_present) {
    out.mapReference = (cpm_MapReference_t*) calloc(1, sizeof(cpm_MapReference_t));
    toStruct_MapReference(in.map_reference, *out.mapReference);
  }
}

}
