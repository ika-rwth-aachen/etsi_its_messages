#pragma once

#include <etsi_its_cam_coding/CamParameters.h>
#include <etsi_its_cam_msgs/CamParameters.h>
#include <etsi_its_cam_conversion/convertBasicContainer.h>
#include <etsi_its_cam_conversion/convertHighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertLowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertSpecialVehicleContainer.h>

namespace etsi_its_cam_conversion {
  
void convert_CamParameterstoRos(const CamParameters_t& _CamParameters_in, etsi_its_cam_msgs::CamParameters& _CamParameters_out) {
  convert_BasicContainertoRos(_CamParameters_in.basicContainer, _CamParameters_out.basicContainer);
  convert_HighFrequencyContainertoRos(_CamParameters_in.highFrequencyContainer, _CamParameters_out.highFrequencyContainer);
  if (_CamParameters_in.lowFrequencyContainer) {
    convert_LowFrequencyContainertoRos(*_CamParameters_in.lowFrequencyContainer, _CamParameters_out.lowFrequencyContainer);
    _CamParameters_out.lowFrequencyContainer_isPresent = true;
  }
  if (_CamParameters_in.specialVehicleContainer) {
    convert_SpecialVehicleContainertoRos(*_CamParameters_in.specialVehicleContainer, _CamParameters_out.specialVehicleContainer);
    _CamParameters_out.specialVehicleContainer_isPresent = true;
  }
}

void convert_CamParameterstoC(const etsi_its_cam_msgs::CamParameters& _CamParameters_in, CamParameters_t& _CamParameters_out) {
  memset(&_CamParameters_out, 0, sizeof(CamParameters_t));
  convert_BasicContainertoC(_CamParameters_in.basicContainer, _CamParameters_out.basicContainer);
  convert_HighFrequencyContainertoC(_CamParameters_in.highFrequencyContainer, _CamParameters_out.highFrequencyContainer);
  if (_CamParameters_in.lowFrequencyContainer_isPresent) {
    LowFrequencyContainer_t lowFrequencyContainer;
    convert_LowFrequencyContainertoC(_CamParameters_in.lowFrequencyContainer, lowFrequencyContainer);
    _CamParameters_out.lowFrequencyContainer = new LowFrequencyContainer_t(lowFrequencyContainer);
  }
  if (_CamParameters_in.specialVehicleContainer_isPresent) {
    SpecialVehicleContainer_t specialVehicleContainer;
    convert_SpecialVehicleContainertoC(_CamParameters_in.specialVehicleContainer, specialVehicleContainer);
    _CamParameters_out.specialVehicleContainer = new SpecialVehicleContainer_t(specialVehicleContainer);
  }
}

}