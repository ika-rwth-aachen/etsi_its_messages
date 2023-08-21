#pragma once

#include <etsi_its_cam_coding/LowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerLowFrequency.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/low_frequency_container.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/LowFrequencyContainer.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_LowFrequencyContainer(const LowFrequencyContainer_t& in, cam_msgs::LowFrequencyContainer& out) {

  if (in.present == LowFrequencyContainer_PR::LowFrequencyContainer_PR_basicVehicleContainerLowFrequency) {
    toRos_BasicVehicleContainerLowFrequency(in.choice.basicVehicleContainerLowFrequency, out.basic_vehicle_container_low_frequency);
    out.choice = cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;
  }
}

void toStruct_LowFrequencyContainer(const cam_msgs::LowFrequencyContainer& in, LowFrequencyContainer_t& out) {
    
  memset(&out, 0, sizeof(LowFrequencyContainer_t));

  if (in.choice == cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY) {
    toStruct_BasicVehicleContainerLowFrequency(in.basic_vehicle_container_low_frequency, out.choice.basicVehicleContainerLowFrequency);
    out.present = LowFrequencyContainer_PR::LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
  }

}

}