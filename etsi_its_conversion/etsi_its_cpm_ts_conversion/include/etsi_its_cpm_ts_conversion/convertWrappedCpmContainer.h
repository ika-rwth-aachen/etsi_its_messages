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

#include <etsi_its_cpm_ts_coding/cpm_ts_WrappedCpmContainer.h>
#include <etsi_its_cpm_ts_conversion/convertCpmContainerId.h>
#include <etsi_its_cpm_ts_conversion/convertOriginatingRsuContainer.h>
#include <etsi_its_cpm_ts_conversion/convertOriginatingVehicleContainer.h>
#include <etsi_its_cpm_ts_conversion/convertPerceivedObjectContainer.h>
#include <etsi_its_cpm_ts_conversion/convertPerceptionRegionContainer.h>
#include <etsi_its_cpm_ts_conversion/convertSensorInformationContainer.h>
#ifdef ROS1
#include <etsi_its_cpm_ts_msgs/WrappedCpmContainer.h>
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs;
#else
#include <etsi_its_cpm_ts_msgs/msg/wrapped_cpm_container.hpp>
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs::msg;
#endif


namespace etsi_its_cpm_ts_conversion {

void toRos_WrappedCpmContainer(const cpm_ts_WrappedCpmContainer_t& in, cpm_ts_msgs::WrappedCpmContainer& out) {
  toRos_CpmContainerId(in.containerId, out.container_id);
  switch (in.containerData.present) {
  case cpm_ts_WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer:
    toRos_OriginatingVehicleContainer(in.containerData.choice.OriginatingVehicleContainer, out.container_data.originating_vehicle_container);
    out.container_data.choice.value = cpm_ts_msgs::CpmContainerId::ORIGINATING_VEHICLE_CONTAINER;
    break;
  case cpm_ts_WrappedCpmContainer__containerData_PR_OriginatingRsuContainer:
    toRos_OriginatingRsuContainer(in.containerData.choice.OriginatingRsuContainer, out.container_data.originating_rsu_container);
    out.container_data.choice.value = cpm_ts_msgs::CpmContainerId::ORIGINATING_RSU_CONTAINER;
    break;
  case cpm_ts_WrappedCpmContainer__containerData_PR_SensorInformationContainer:
    toRos_SensorInformationContainer(in.containerData.choice.SensorInformationContainer, out.container_data.sensor_information_container);
    out.container_data.choice.value = cpm_ts_msgs::CpmContainerId::SENSOR_INFORMATION_CONTAINER;
    break;
  case cpm_ts_WrappedCpmContainer__containerData_PR_PerceptionRegionContainer:
    toRos_PerceptionRegionContainer(in.containerData.choice.PerceptionRegionContainer, out.container_data.perception_region_container);
    out.container_data.choice.value = cpm_ts_msgs::CpmContainerId::PERCEPTION_REGION_CONTAINER;
    break;
  case cpm_ts_WrappedCpmContainer__containerData_PR_PerceivedObjectContainer:
    toRos_PerceivedObjectContainer(in.containerData.choice.PerceivedObjectContainer, out.container_data.perceived_object_container);
    out.container_data.choice.value = cpm_ts_msgs::CpmContainerId::PERCEIVED_OBJECT_CONTAINER;
    break;
  }
}

void toStruct_WrappedCpmContainer(const cpm_ts_msgs::WrappedCpmContainer& in, cpm_ts_WrappedCpmContainer_t& out) {
  memset(&out, 0, sizeof(cpm_ts_WrappedCpmContainer_t));

  toStruct_CpmContainerId(in.container_id, out.containerId);
  switch (in.container_data.choice.value) {
  case cpm_ts_msgs::CpmContainerId::ORIGINATING_VEHICLE_CONTAINER:
    toStruct_OriginatingVehicleContainer(in.container_data.originating_vehicle_container, out.containerData.choice.OriginatingVehicleContainer);
    out.containerData.present = cpm_ts_WrappedCpmContainer__containerData_PR::cpm_ts_WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer;
    break;
  case cpm_ts_msgs::CpmContainerId::ORIGINATING_RSU_CONTAINER:
    toStruct_OriginatingRsuContainer(in.container_data.originating_rsu_container, out.containerData.choice.OriginatingRsuContainer);
    out.containerData.present = cpm_ts_WrappedCpmContainer__containerData_PR::cpm_ts_WrappedCpmContainer__containerData_PR_OriginatingRsuContainer;
    break;
  case cpm_ts_msgs::CpmContainerId::SENSOR_INFORMATION_CONTAINER:
    toStruct_SensorInformationContainer(in.container_data.sensor_information_container, out.containerData.choice.SensorInformationContainer);
    out.containerData.present = cpm_ts_WrappedCpmContainer__containerData_PR::cpm_ts_WrappedCpmContainer__containerData_PR_SensorInformationContainer;
    break;
  case cpm_ts_msgs::CpmContainerId::PERCEPTION_REGION_CONTAINER:
    toStruct_PerceptionRegionContainer(in.container_data.perception_region_container, out.containerData.choice.PerceptionRegionContainer);
    out.containerData.present = cpm_ts_WrappedCpmContainer__containerData_PR::cpm_ts_WrappedCpmContainer__containerData_PR_PerceptionRegionContainer;
    break;
  case cpm_ts_msgs::CpmContainerId::PERCEIVED_OBJECT_CONTAINER:
    toStruct_PerceivedObjectContainer(in.container_data.perceived_object_container, out.containerData.choice.PerceivedObjectContainer);
    out.containerData.present = cpm_ts_WrappedCpmContainer__containerData_PR::cpm_ts_WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;
    break;
  }
}

}
