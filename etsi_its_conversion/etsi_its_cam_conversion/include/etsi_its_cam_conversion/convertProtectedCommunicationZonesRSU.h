#pragma once

#include <etsi_its_cam_coding/ProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZonesRSU.h>

namespace etsi_its_cam_conversion
{
	void convert_ProtectedCommunicationZonesRSUtoRos(const ProtectedCommunicationZonesRSU_t& _ProtectedCommunicationZonesRSU_in, etsi_its_cam_msgs::ProtectedCommunicationZonesRSU& _ProtectedCommunicationZonesRSU_out)
	{
	}
	void convert_ProtectedCommunicationZonesRSUtoC(const etsi_its_cam_msgs::ProtectedCommunicationZonesRSU& _ProtectedCommunicationZonesRSU_in, ProtectedCommunicationZonesRSU_t& _ProtectedCommunicationZonesRSU_out)
	{
		memset(&_ProtectedCommunicationZonesRSU_out, 0, sizeof(ProtectedCommunicationZonesRSU_t));
	}
}