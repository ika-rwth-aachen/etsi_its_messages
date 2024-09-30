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

#include <etsi_its_spatem_ts_coding/spatem_ts_LaneAttributes-addGrpC.h>
#include <etsi_its_spatem_ts_conversion/convertVehicleHeight.h>
#include <etsi_its_spatem_ts_conversion/convertVehicleMass.h>
#ifdef ROS1
#include <etsi_its_spatem_ts_msgs/LaneAttributesAddGrpC.h>
namespace spatem_ts_msgs = etsi_its_spatem_ts_msgs;
#else
#include <etsi_its_spatem_ts_msgs/msg/lane_attributes_add_grp_c.hpp>
namespace spatem_ts_msgs = etsi_its_spatem_ts_msgs::msg;
#endif


namespace etsi_its_spatem_ts_conversion {

void toRos_LaneAttributesAddGrpC(const spatem_ts_LaneAttributes_addGrpC_t& in, spatem_ts_msgs::LaneAttributesAddGrpC& out) {
  if (in.maxVehicleHeight) {
    toRos_VehicleHeight(*in.maxVehicleHeight, out.max_vehicle_height);
    out.max_vehicle_height_is_present = true;
  }
  if (in.maxVehicleWeight) {
    toRos_VehicleMass(*in.maxVehicleWeight, out.max_vehicle_weight);
    out.max_vehicle_weight_is_present = true;
  }
}

void toStruct_LaneAttributesAddGrpC(const spatem_ts_msgs::LaneAttributesAddGrpC& in, spatem_ts_LaneAttributes_addGrpC_t& out) {
  memset(&out, 0, sizeof(spatem_ts_LaneAttributes_addGrpC_t));

  if (in.max_vehicle_height_is_present) {
    out.maxVehicleHeight = (spatem_ts_VehicleHeight_t*) calloc(1, sizeof(spatem_ts_VehicleHeight_t));
    toStruct_VehicleHeight(in.max_vehicle_height, *out.maxVehicleHeight);
  }
  if (in.max_vehicle_weight_is_present) {
    out.maxVehicleWeight = (spatem_ts_VehicleMass_t*) calloc(1, sizeof(spatem_ts_VehicleMass_t));
    toStruct_VehicleMass(in.max_vehicle_weight, *out.maxVehicleWeight);
  }
}

}
