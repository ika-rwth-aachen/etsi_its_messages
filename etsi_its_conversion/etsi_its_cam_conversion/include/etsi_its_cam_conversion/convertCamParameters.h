#pragma once

#include <etsi_its_cam_coding/CamParameters.h>
#include <etsi_its_cam_conversion/convertBasicContainer.h>
#include <etsi_its_cam_conversion/convertHighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertLowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertSpecialVehicleContainer.h>
#include <etsi_its_cam_msgs/CamParameters.h>


namespace etsi_its_cam_conversion {

void toRos_CamParameters(const CamParameters_t& in, etsi_its_cam_msgs::CamParameters& out) {

  toRos_BasicContainer(in.basic_container, out.basic_container);
  toRos_HighFrequencyContainer(in.high_frequency_container, out.high_frequency_container);
  if (in.low_frequency_container) {
    toRos_LowFrequencyContainer(*in.low_frequency_container, out.low_frequency_container);
    out.low_frequency_container_is_present = true;
  }

  if (in.special_vehicle_container) {
    toRos_SpecialVehicleContainer(*in.special_vehicle_container, out.special_vehicle_container);
    out.special_vehicle_container_is_present = true;
  }

}

void toStruct_CamParameters(const etsi_its_cam_msgs::CamParameters& in, CamParameters_t& out) {
    
  memset(&out, 0, sizeof(CamParameters_t));

  toStruct_BasicContainer(in.basic_container, out.basic_container);
  toStruct_HighFrequencyContainer(in.high_frequency_container, out.high_frequency_container);
  if (in.low_frequency_container_is_present) {
    LowFrequencyContainer_t low_frequency_container;
    toStruct_LowFrequencyContainer(in.low_frequency_container, low_frequency_container);
    out.low_frequency_container = new LowFrequencyContainer_t(low_frequency_container);
  }

  if (in.special_vehicle_container_is_present) {
    SpecialVehicleContainer_t special_vehicle_container;
    toStruct_SpecialVehicleContainer(in.special_vehicle_container, special_vehicle_container);
    out.special_vehicle_container = new SpecialVehicleContainer_t(special_vehicle_container);
  }

}

}