#pragma once

#include <etsi_its_cam_coding/EmergencyContainer.h>
#include <etsi_its_cam_msgs/EmergencyContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertEmergencyPriority.h>

namespace etsi_its_cam_conversion
{
	void convert_EmergencyContainertoRos(const EmergencyContainer_t& _EmergencyContainer_in, etsi_its_cam_msgs::EmergencyContainer& _EmergencyContainer_out)
	{
		convert_LightBarSirenInUsetoRos(_EmergencyContainer_in.lightBarSirenInUse, _EmergencyContainer_out.lightBarSirenInUse);
		if(_EmergencyContainer_in.incidentIndication)
		{
			convert_CauseCodetoRos(*_EmergencyContainer_in.incidentIndication, _EmergencyContainer_out.incidentIndication);
			_EmergencyContainer_out.incidentIndication_isPresent = true;
		}
		if(_EmergencyContainer_in.emergencyPriority)
		{
			convert_EmergencyPrioritytoRos(*_EmergencyContainer_in.emergencyPriority, _EmergencyContainer_out.emergencyPriority);
			_EmergencyContainer_out.emergencyPriority_isPresent = true;
		}
	}
	void convert_EmergencyContainertoC(const etsi_its_cam_msgs::EmergencyContainer& _EmergencyContainer_in, EmergencyContainer_t& _EmergencyContainer_out)
	{
		memset(&_EmergencyContainer_out, 0, sizeof(EmergencyContainer_t));
		convert_LightBarSirenInUsetoC(_EmergencyContainer_in.lightBarSirenInUse, _EmergencyContainer_out.lightBarSirenInUse);
		if(_EmergencyContainer_in.incidentIndication_isPresent)
		{
			CauseCode_t incidentIndication;
			convert_CauseCodetoC(_EmergencyContainer_in.incidentIndication, incidentIndication);
			_EmergencyContainer_out.incidentIndication = new CauseCode_t(incidentIndication);
		}
		if(_EmergencyContainer_in.emergencyPriority_isPresent)
		{
			EmergencyPriority_t emergencyPriority;
			convert_EmergencyPrioritytoC(_EmergencyContainer_in.emergencyPriority, emergencyPriority);
			_EmergencyContainer_out.emergencyPriority = new EmergencyPriority_t(emergencyPriority);
		}
	}
}