/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include <etsi_its_spatem_coding/RequestorType.h>
#include <etsi_its_spatem_conversion/convertBasicVehicleRole.h>
#include <etsi_its_spatem_conversion/convertRequestSubRole.h>
#include <etsi_its_spatem_conversion/convertRequestImportanceLevel.h>
#include <etsi_its_spatem_conversion/convertIso3833VehicleType.h>
#include <etsi_its_spatem_conversion/convertVehicleType.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/RequestorType.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/requestor_type.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_RequestorType(const RequestorType_t& in, spatem_msgs::RequestorType& out) {

  toRos_BasicVehicleRole(in.role, out.role);
  if (in.subrole) {
    toRos_RequestSubRole(*in.subrole, out.subrole);
    out.subrole_is_present = true;
  }

  if (in.request) {
    toRos_RequestImportanceLevel(*in.request, out.request);
    out.request_is_present = true;
  }

  if (in.iso3883) {
    toRos_Iso3833VehicleType(*in.iso3883, out.iso_3883);
    out.iso_3883_is_present = true;
  }

  if (in.hpmsType) {
    toRos_VehicleType(*in.hpmsType, out.hpms_type);
    out.hpms_type_is_present = true;
  }

  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_RequestorType(const spatem_msgs::RequestorType& in, RequestorType_t& out) {

  memset(&out, 0, sizeof(RequestorType_t));

  toStruct_BasicVehicleRole(in.role, out.role);
  if (in.subrole_is_present) {
    RequestSubRole_t subrole;
    toStruct_RequestSubRole(in.subrole, subrole);
    out.subrole = new RequestSubRole_t(subrole);
  }

  if (in.request_is_present) {
    RequestImportanceLevel_t request;
    toStruct_RequestImportanceLevel(in.request, request);
    out.request = new RequestImportanceLevel_t(request);
  }

  if (in.iso_3883_is_present) {
    Iso3833VehicleType_t iso_3883;
    toStruct_Iso3833VehicleType(in.iso_3883, iso_3883);
    out.iso3883 = new Iso3833VehicleType_t(iso_3883);
  }

  if (in.hpms_type_is_present) {
    VehicleType_t hpms_type;
    toStruct_VehicleType(in.hpms_type, hpms_type);
    out.hpmsType = new VehicleType_t(hpms_type);
  }

  if (in.regional_is_present) {
    RegionalExtension_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_t(regional);
  }

}

}