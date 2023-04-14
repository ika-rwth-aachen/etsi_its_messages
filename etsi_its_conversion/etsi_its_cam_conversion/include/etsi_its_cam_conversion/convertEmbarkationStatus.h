#pragma once

#include <etsi_its_cam_coding/EmbarkationStatus.h>
#include <etsi_its_cam_msgs/EmbarkationStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBOOLEAN.h>

namespace etsi_its_cam_conversion
{
	void convert_EmbarkationStatustoRos(const EmbarkationStatus_t& _EmbarkationStatus_in, etsi_its_cam_msgs::EmbarkationStatus& _EmbarkationStatus_out)
	{
		convert_BOOLEANtoRos(_EmbarkationStatus_in, _EmbarkationStatus_out.value);
	}
	void convert_EmbarkationStatustoC(const etsi_its_cam_msgs::EmbarkationStatus& _EmbarkationStatus_in, EmbarkationStatus_t& _EmbarkationStatus_out)
	{
		memset(&_EmbarkationStatus_out, 0, sizeof(EmbarkationStatus_t));
		convert_BOOLEANtoC(_EmbarkationStatus_in.value, _EmbarkationStatus_out);
	}
}