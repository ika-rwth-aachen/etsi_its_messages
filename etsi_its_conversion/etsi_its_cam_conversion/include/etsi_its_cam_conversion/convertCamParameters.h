//// SEQUENCE CamParameters


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/CamParameters.h>
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

void toRos_CamParameters(const CamParameters_t& in, cam_msgs::CamParameters& out) {
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

void toStruct_CamParameters(const cam_msgs::CamParameters& in, CamParameters_t& out) {
  memset(&out, 0, sizeof(CamParameters_t));

  toStruct_BasicContainer(in.basic_container, out.basicContainer);
  toStruct_HighFrequencyContainer(in.high_frequency_container, out.highFrequencyContainer);
  if (in.low_frequency_container_is_present) {
    out.lowFrequencyContainer = (LowFrequencyContainer_t*) calloc(1, sizeof(LowFrequencyContainer_t));
    toStruct_LowFrequencyContainer(in.low_frequency_container, *out.lowFrequencyContainer);
  }
  if (in.special_vehicle_container_is_present) {
    out.specialVehicleContainer = (SpecialVehicleContainer_t*) calloc(1, sizeof(SpecialVehicleContainer_t));
    toStruct_SpecialVehicleContainer(in.special_vehicle_container, *out.specialVehicleContainer);
  }
}

}
