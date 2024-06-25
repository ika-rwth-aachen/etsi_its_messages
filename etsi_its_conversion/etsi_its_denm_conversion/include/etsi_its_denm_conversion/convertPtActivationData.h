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

#include <etsi_its_denm_coding/denm_PtActivationData.h>
#include <etsi_its_denm_coding/OCTET_STRING.h>
#include <etsi_its_primitives_conversion/convertOCTET_STRING.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PtActivationData.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/pt_activation_data.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PtActivationData(const denm_PtActivationData_t& in, denm_msgs::PtActivationData& out) {
  etsi_its_primitives_conversion::toRos_OCTET_STRING(in, out.value);
}

void toStruct_PtActivationData(const denm_msgs::PtActivationData& in, denm_PtActivationData_t& out) {
  memset(&out, 0, sizeof(denm_PtActivationData_t));

  etsi_its_primitives_conversion::toStruct_OCTET_STRING(in.value, out);
}

}
