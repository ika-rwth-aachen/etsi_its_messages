#pragma once

#include <BasicVehicleContainerLowFrequency.h>
#include <etsi_its_cam_msgs/BasicVehicleContainerLowFrequency.h>
#include <convertVehicleRole.h>
#include <convertExteriorLights.h>
#include <convertPathHistory.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::BasicVehicleContainerLowFrequency convert_BasicVehicleContainerLowFrequencytoRos(const BasicVehicleContainerLowFrequency_t& _BasicVehicleContainerLowFrequency_in)
	{
		etsi_its_cam_msgs::BasicVehicleContainerLowFrequency BasicVehicleContainerLowFrequency_out;
		BasicVehicleContainerLowFrequency_out.vehicleRole = convert_VehicleRoletoRos(_BasicVehicleContainerLowFrequency_in.vehicleRole);
		BasicVehicleContainerLowFrequency_out.exteriorLights = convert_ExteriorLightstoRos(_BasicVehicleContainerLowFrequency_in.exteriorLights);
		BasicVehicleContainerLowFrequency_out.pathHistory = convert_PathHistorytoRos(_BasicVehicleContainerLowFrequency_in.pathHistory);
		return BasicVehicleContainerLowFrequency_out;
	}
}