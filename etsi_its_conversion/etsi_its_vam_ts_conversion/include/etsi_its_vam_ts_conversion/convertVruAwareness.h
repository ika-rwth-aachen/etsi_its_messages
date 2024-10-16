/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
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

#include <etsi_its_vam_ts_coding/vam_ts_VruAwareness.h>
#include <etsi_its_vam_ts_conversion/convertGenerationDeltaTime.h>
#include <etsi_its_vam_ts_conversion/convertVamParameters.h>
#ifdef ROS1
#include <etsi_its_vam_ts_msgs/VruAwareness.h>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs;
#else
#include <etsi_its_vam_ts_msgs/msg/vru_awareness.hpp>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs::msg;
#endif


namespace etsi_its_vam_ts_conversion {

void toRos_VruAwareness(const vam_ts_VruAwareness_t& in, vam_ts_msgs::VruAwareness& out) {
  toRos_GenerationDeltaTime(in.generationDeltaTime, out.generation_delta_time);
  toRos_VamParameters(in.vamParameters, out.vam_parameters);
}

void toStruct_VruAwareness(const vam_ts_msgs::VruAwareness& in, vam_ts_VruAwareness_t& out) {
  memset(&out, 0, sizeof(vam_ts_VruAwareness_t));

  toStruct_GenerationDeltaTime(in.generation_delta_time, out.generationDeltaTime);
  toStruct_VamParameters(in.vam_parameters, out.vamParameters);
}

}