#pragma once

#include <etsi_its_cam_coding/LowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerLowFrequency.h>
#include <etsi_its_cam_msgs/LowFrequencyContainer.h>


namespace etsi_its_cam_conversion {

void toRos_LowFrequencyContainer(const LowFrequencyContainer_t& in, etsi_its_cam_msgs::LowFrequencyContainer& out) {

  if (in.present == LowFrequencyContainer_PR::LowFrequencyContainer_PR_basic_vehicle_container_low_frequency) {
    toRos_BasicVehicleContainerLowFrequency(in.choice.basic_vehicle_container_low_frequency, out.basic_vehicle_container_low_frequency);
    out.choice = etsi_its_cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;
  }
}

void toStruct_LowFrequencyContainer(const etsi_its_cam_msgs::LowFrequencyContainer& in, LowFrequencyContainer_t& out) {
    
  memset(&out, 0, sizeof(LowFrequencyContainer_t));

  if (in.choice == etsi_its_cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY) {
    toStruct_BasicVehicleContainerLowFrequency(in.basic_vehicle_container_low_frequency, out.choice.basic_vehicle_container_low_frequency);
    out.present = LowFrequencyContainer_PR::LowFrequencyContainer_PR_basic_vehicle_container_low_frequency;
  }

}

}