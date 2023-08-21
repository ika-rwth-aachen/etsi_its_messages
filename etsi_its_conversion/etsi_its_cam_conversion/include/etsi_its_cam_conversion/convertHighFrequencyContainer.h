#pragma once

#include <etsi_its_cam_coding/HighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertRSUContainerHighFrequency.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/high_frequency_container.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/HighFrequencyContainer.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_HighFrequencyContainer(const HighFrequencyContainer_t& in, cam_msgs::HighFrequencyContainer& out) {

  if (in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
    toRos_BasicVehicleContainerHighFrequency(in.choice.basicVehicleContainerHighFrequency, out.basic_vehicle_container_high_frequency);
    out.choice = cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
  }

  if (in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency) {
    toRos_RSUContainerHighFrequency(in.choice.rsuContainerHighFrequency, out.rsu_container_high_frequency);
    out.choice = cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
  }
}

void toStruct_HighFrequencyContainer(const cam_msgs::HighFrequencyContainer& in, HighFrequencyContainer_t& out) {
    
  memset(&out, 0, sizeof(HighFrequencyContainer_t));

  if (in.choice == cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY) {
    toStruct_BasicVehicleContainerHighFrequency(in.basic_vehicle_container_high_frequency, out.choice.basicVehicleContainerHighFrequency);
    out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
  }

  if (in.choice == cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY) {
    toStruct_RSUContainerHighFrequency(in.rsu_container_high_frequency, out.choice.rsuContainerHighFrequency);
    out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency;
  }

}

}