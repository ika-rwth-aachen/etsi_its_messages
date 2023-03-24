#pragma once

#include <etsi_its_cam_coding/ProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZonesRSU.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ProtectedCommunicationZonesRSU convert_ProtectedCommunicationZonesRSUtoRos(const ProtectedCommunicationZonesRSU_t& _ProtectedCommunicationZonesRSU_in)
	{
		etsi_its_cam_msgs::ProtectedCommunicationZonesRSU ProtectedCommunicationZonesRSU_out;
		return ProtectedCommunicationZonesRSU_out;
	}
}