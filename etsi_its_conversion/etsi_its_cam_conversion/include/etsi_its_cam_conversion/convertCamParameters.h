#pragma once

#include <etsi_its_cam_coding/CamParameters.h>
#include <etsi_its_cam_msgs/CamParameters.h>
#include <etsi_its_cam_conversion/convertBasicContainer.h>
#include <etsi_its_cam_conversion/convertHighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertLowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertSpecialVehicleContainer.h>

namespace etsi_its_cam_conversion {
  
void toRos_CamParameters(const CamParameters_t& in, etsi_its_cam_msgs::CamParameters& out) {
  toRos_BasicContainer(in.basicContainer, out.basicContainer);
  toRos_HighFrequencyContainer(in.highFrequencyContainer, out.highFrequencyContainer);
  if (in.lowFrequencyContainer) {
    toRos_LowFrequencyContainer(*in.lowFrequencyContainer, out.lowFrequencyContainer);
    out.lowFrequencyContainer_isPresent = true;
  }
  if (in.specialVehicleContainer) {
    toRos_SpecialVehicleContainer(*in.specialVehicleContainer, out.specialVehicleContainer);
    out.specialVehicleContainer_isPresent = true;
  }
}

void toStruct_CamParameters(const etsi_its_cam_msgs::CamParameters& in, CamParameters_t& out) {
  memset(&out, 0, sizeof(CamParameters_t));
  toStruct_BasicContainer(in.basicContainer, out.basicContainer);
  toStruct_HighFrequencyContainer(in.highFrequencyContainer, out.highFrequencyContainer);
  if (in.lowFrequencyContainer_isPresent) {
    LowFrequencyContainer_t lowFrequencyContainer;
    toStruct_LowFrequencyContainer(in.lowFrequencyContainer, lowFrequencyContainer);
    out.lowFrequencyContainer = new LowFrequencyContainer_t(lowFrequencyContainer);
  }
  if (in.specialVehicleContainer_isPresent) {
    SpecialVehicleContainer_t specialVehicleContainer;
    toStruct_SpecialVehicleContainer(in.specialVehicleContainer, specialVehicleContainer);
    out.specialVehicleContainer = new SpecialVehicleContainer_t(specialVehicleContainer);
  }
}

}