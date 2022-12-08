#pragma once

#include <RSUContainerHighFrequency.h>
#include <etsi_its_cam_msgs/RSUContainerHighFrequency.h>
#include <convertProtectedCommunicationZonesRSU.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::RSUContainerHighFrequency convert_RSUContainerHighFrequencytoRos(const RSUContainerHighFrequency_t& _RSUContainerHighFrequency_in)
	{
		etsi_its_cam_msgs::RSUContainerHighFrequency RSUContainerHighFrequency_out;
		if(_RSUContainerHighFrequency_in.protectedCommunicationZonesRSU)
		{
			RSUContainerHighFrequency_out.protectedCommunicationZonesRSU = convert_ProtectedCommunicationZonesRSUtoRos(*_RSUContainerHighFrequency_in.protectedCommunicationZonesRSU);
			RSUContainerHighFrequency_out.protectedCommunicationZonesRSU_isPresent = true;
		}
		return RSUContainerHighFrequency_out;
	}
}