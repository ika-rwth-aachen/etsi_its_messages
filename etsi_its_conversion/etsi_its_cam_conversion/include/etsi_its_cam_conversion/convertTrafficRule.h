#pragma once

#include <etsi_its_cam_coding/TrafficRule.h>
#include <etsi_its_cam_msgs/TrafficRule.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::TrafficRule convert_TrafficRuletoRos(const TrafficRule_t& _TrafficRule_in)
	{
		etsi_its_cam_msgs::TrafficRule TrafficRule_out;
		TrafficRule_out.value = _TrafficRule_in;
		return TrafficRule_out;
	}
	TrafficRule_t convert_TrafficRuletoC(const etsi_its_cam_msgs::TrafficRule& _TrafficRule_in)
	{
		TrafficRule_t TrafficRule_out;
		TrafficRule_out = _TrafficRule_in.value;
		return TrafficRule_out;
	}
}