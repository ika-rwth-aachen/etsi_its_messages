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

#include <etsi_its_cam_coding/cam_CamParameters.h>
#include <etsi_its_cam_conversion/convertBasicContainer.h>
#include <etsi_its_cam_conversion/convertHighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertLowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertSpecialVehicleContainer.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CamParameters.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/cam_parameters.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CamParameters(const cam_CamParameters_t& in, cam_msgs::CamParameters& out) {
  toRos_BasicContainer(in.basicContainer, out.basic_container);
  toRos_HighFrequencyContainer(in.highFrequencyContainer, out.high_frequency_container);
  if (in.lowFrequencyContainer) {
    toRos_LowFrequencyContainer(*in.lowFrequencyContainer, out.low_frequency_container);
    out.low_frequency_container_is_present = true;
  }
  if (in.specialVehicleContainer) {
    toRos_SpecialVehicleContainer(*in.specialVehicleContainer, out.special_vehicle_container);
    out.special_vehicle_container_is_present = true;
  }
}

void toStruct_CamParameters(const cam_msgs::CamParameters& in, cam_CamParameters_t& out) {
  memset(&out, 0, sizeof(cam_CamParameters_t));

  toStruct_BasicContainer(in.basic_container, out.basicContainer);
  toStruct_HighFrequencyContainer(in.high_frequency_container, out.highFrequencyContainer);
  if (in.low_frequency_container_is_present) {
    out.lowFrequencyContainer = (cam_LowFrequencyContainer_t*) calloc(1, sizeof(cam_LowFrequencyContainer_t));
    toStruct_LowFrequencyContainer(in.low_frequency_container, *out.lowFrequencyContainer);
  }
  if (in.special_vehicle_container_is_present) {
    out.specialVehicleContainer = (cam_SpecialVehicleContainer_t*) calloc(1, sizeof(cam_SpecialVehicleContainer_t));
    toStruct_SpecialVehicleContainer(in.special_vehicle_container, *out.specialVehicleContainer);
  }
}

}
