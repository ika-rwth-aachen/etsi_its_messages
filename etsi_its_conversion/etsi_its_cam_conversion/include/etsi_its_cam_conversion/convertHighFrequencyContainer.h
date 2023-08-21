#pragma once

#include <etsi_its_cam_coding/HighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertRSUContainerHighFrequency.h>
#include <etsi_its_cam_msgs/HighFrequencyContainer.h>


namespace etsi_its_cam_conversion {

void toRos_HighFrequencyContainer(const HighFrequencyContainer_t& in, etsi_its_cam_msgs::HighFrequencyContainer& out) {

  if (in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_basic_vehicle_container_high_frequency) {
    toRos_BasicVehicleContainerHighFrequency(in.choice.basic_vehicle_container_high_frequency, out.basic_vehicle_container_high_frequency);
    out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
  }

  if (in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsu_container_high_frequency) {
    toRos_RSUContainerHighFrequency(in.choice.rsu_container_high_frequency, out.rsu_container_high_frequency);
    out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
  }
}

void toStruct_HighFrequencyContainer(const etsi_its_cam_msgs::HighFrequencyContainer& in, HighFrequencyContainer_t& out) {
    
  memset(&out, 0, sizeof(HighFrequencyContainer_t));

  if (in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY) {
    toStruct_BasicVehicleContainerHighFrequency(in.basic_vehicle_container_high_frequency, out.choice.basic_vehicle_container_high_frequency);
    out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_basic_vehicle_container_high_frequency;
  }

  if (in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY) {
    toStruct_RSUContainerHighFrequency(in.rsu_container_high_frequency, out.choice.rsu_container_high_frequency);
    out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsu_container_high_frequency;
  }

}

}