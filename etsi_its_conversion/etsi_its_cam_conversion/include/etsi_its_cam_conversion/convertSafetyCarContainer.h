#pragma once

#include <etsi_its_cam_coding/SafetyCarContainer.h>
#include <etsi_its_cam_msgs/SafetyCarContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertTrafficRule.h>
#include <etsi_its_cam_conversion/convertSpeedLimit.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SafetyCarContainer convert_SafetyCarContainertoRos(const SafetyCarContainer_t& _SafetyCarContainer_in)
	{
		etsi_its_cam_msgs::SafetyCarContainer SafetyCarContainer_out;
		SafetyCarContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_SafetyCarContainer_in.lightBarSirenInUse);
		if(_SafetyCarContainer_in.incidentIndication)
		{
			SafetyCarContainer_out.incidentIndication = convert_CauseCodetoRos(*_SafetyCarContainer_in.incidentIndication);
			SafetyCarContainer_out.incidentIndication_isPresent = true;
		}
		if(_SafetyCarContainer_in.trafficRule)
		{
			SafetyCarContainer_out.trafficRule = convert_TrafficRuletoRos(*_SafetyCarContainer_in.trafficRule);
			SafetyCarContainer_out.trafficRule_isPresent = true;
		}
		if(_SafetyCarContainer_in.speedLimit)
		{
			SafetyCarContainer_out.speedLimit = convert_SpeedLimittoRos(*_SafetyCarContainer_in.speedLimit);
			SafetyCarContainer_out.speedLimit_isPresent = true;
		}
		return SafetyCarContainer_out;
	}
}