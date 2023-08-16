#pragma once

#include <etsi_its_cam_coding/HighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertRSUContainerHighFrequency.h>
#include <etsi_its_cam_msgs/HighFrequencyContainer.h>


namespace etsi_its_cam_conversion {

void toRos_HighFrequencyContainer(const HighFrequencyContainer_t& in, etsi_its_cam_msgs::HighFrequencyContainer& out) {

  if (in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
    toRos_BasicVehicleContainerHighFrequency(in.choice.basicVehicleContainerHighFrequency, out.basicVehicleContainerHighFrequency);
    out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
  }

  if (in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency) {
    toRos_RSUContainerHighFrequency(in.choice.rsuContainerHighFrequency, out.rsuContainerHighFrequency);
    out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
  }
}

void toStruct_HighFrequencyContainer(const etsi_its_cam_msgs::HighFrequencyContainer& in, HighFrequencyContainer_t& out) {
    
  memset(&out, 0, sizeof(HighFrequencyContainer_t));

  if (in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY) {
    toStruct_BasicVehicleContainerHighFrequency(in.basicVehicleContainerHighFrequency, out.choice.basicVehicleContainerHighFrequency);
    out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
  }

  if (in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY) {
    toStruct_RSUContainerHighFrequency(in.rsuContainerHighFrequency, out.choice.rsuContainerHighFrequency);
    out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency;
  }

}

}