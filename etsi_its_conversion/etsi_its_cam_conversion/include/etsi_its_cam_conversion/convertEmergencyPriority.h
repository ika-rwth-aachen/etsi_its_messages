#pragma once

#include <etsi_its_cam_coding/EmergencyPriority.h>
#include <etsi_its_cam_msgs/EmergencyPriority.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::EmergencyPriority convert_EmergencyPrioritytoRos(const EmergencyPriority_t& _EmergencyPriority_in)
	{
		etsi_its_cam_msgs::EmergencyPriority EmergencyPriority_out;
		convert_BIT_STRINGtoRos(_EmergencyPriority_in, EmergencyPriority_out.value);
		return EmergencyPriority_out;
	}
	EmergencyPriority_t convert_EmergencyPrioritytoC(const etsi_its_cam_msgs::EmergencyPriority& _EmergencyPriority_in)
	{
		EmergencyPriority_t EmergencyPriority_out;
		convert_BIT_STRINGtoC(_EmergencyPriority_in.value, EmergencyPriority_out);
		return EmergencyPriority_out;
	}
}