#pragma once

#include <EmergencyContainer.h>
#include <etsi_its_cam_msgs/EmergencyContainer.h>
#include <convertLightBarSirenInUse.h>
#include <convertCauseCode.h>
#include <convertEmergencyPriority.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::EmergencyContainer convert_EmergencyContainertoRos(const EmergencyContainer_t& _EmergencyContainer_in)
	{
		etsi_its_cam_msgs::EmergencyContainer EmergencyContainer_out;
		EmergencyContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_EmergencyContainer_in.lightBarSirenInUse);
		if(_EmergencyContainer_in.incidentIndication)
		{
			EmergencyContainer_out.incidentIndication = convert_CauseCodetoRos(*_EmergencyContainer_in.incidentIndication);
		}
		if(_EmergencyContainer_in.emergencyPriority)
		{
			EmergencyContainer_out.emergencyPriority = convert_EmergencyPrioritytoRos(*_EmergencyContainer_in.emergencyPriority);
		}
		return EmergencyContainer_out;
	}
}