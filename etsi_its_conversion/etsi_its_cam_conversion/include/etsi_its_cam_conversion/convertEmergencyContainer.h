#pragma once

#include <etsi_its_cam_coding/EmergencyContainer.h>
#include <etsi_its_cam_msgs/EmergencyContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertEmergencyPriority.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::EmergencyContainer convert_EmergencyContainertoRos(const EmergencyContainer_t& _EmergencyContainer_in)
	{
		etsi_its_cam_msgs::EmergencyContainer EmergencyContainer_out;
		EmergencyContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_EmergencyContainer_in.lightBarSirenInUse);
		if(_EmergencyContainer_in.incidentIndication)
		{
			EmergencyContainer_out.incidentIndication = convert_CauseCodetoRos(*_EmergencyContainer_in.incidentIndication);
			EmergencyContainer_out.incidentIndication_isPresent = true;
		}
		if(_EmergencyContainer_in.emergencyPriority)
		{
			EmergencyContainer_out.emergencyPriority = convert_EmergencyPrioritytoRos(*_EmergencyContainer_in.emergencyPriority);
			EmergencyContainer_out.emergencyPriority_isPresent = true;
		}
		return EmergencyContainer_out;
	}
	EmergencyContainer_t convert_EmergencyContainertoC(const etsi_its_cam_msgs::EmergencyContainer& _EmergencyContainer_in)
	{
		EmergencyContainer_t EmergencyContainer_out;
		EmergencyContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoC(_EmergencyContainer_in.lightBarSirenInUse);
		if(_EmergencyContainer_in.incidentIndication_isPresent)
		{
			auto incidentIndication = convert_CauseCodetoC(_EmergencyContainer_in.incidentIndication);
			EmergencyContainer_out.incidentIndication = &incidentIndication;
		}
		if(_EmergencyContainer_in.emergencyPriority_isPresent)
		{
			auto emergencyPriority = convert_EmergencyPrioritytoC(_EmergencyContainer_in.emergencyPriority);
			EmergencyContainer_out.emergencyPriority = &emergencyPriority;
		}
		return EmergencyContainer_out;
	}
}