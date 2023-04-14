#pragma once

#include <etsi_its_cam_coding/LowFrequencyContainer.h>
#include <etsi_its_cam_msgs/LowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerLowFrequency.h>

namespace etsi_its_cam_conversion {
  
void convert_LowFrequencyContainertoRos(const LowFrequencyContainer_t& _LowFrequencyContainer_in, etsi_its_cam_msgs::LowFrequencyContainer& _LowFrequencyContainer_out) {
  if(_LowFrequencyContainer_in.present == LowFrequencyContainer_PR::LowFrequencyContainer_PR_basicVehicleContainerLowFrequency)
  {
    convert_BasicVehicleContainerLowFrequencytoRos(_LowFrequencyContainer_in.choice.basicVehicleContainerLowFrequency, _LowFrequencyContainer_out.basicVehicleContainerLowFrequency);
    _LowFrequencyContainer_out.choice = etsi_its_cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;
  }
}

void convert_LowFrequencyContainertoC(const etsi_its_cam_msgs::LowFrequencyContainer& _LowFrequencyContainer_in, LowFrequencyContainer_t& _LowFrequencyContainer_out) {
  memset(&_LowFrequencyContainer_out, 0, sizeof(LowFrequencyContainer_t));
  if(_LowFrequencyContainer_in.choice == etsi_its_cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY)
  {
    convert_BasicVehicleContainerLowFrequencytoC(_LowFrequencyContainer_in.basicVehicleContainerLowFrequency, _LowFrequencyContainer_out.choice.basicVehicleContainerLowFrequency);
    _LowFrequencyContainer_out.present = LowFrequencyContainer_PR::LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
  }
}

}