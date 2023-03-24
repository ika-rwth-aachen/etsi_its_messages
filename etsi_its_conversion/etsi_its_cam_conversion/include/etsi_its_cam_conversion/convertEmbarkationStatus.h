#pragma once

#include <etsi_its_cam_coding/EmbarkationStatus.h>
#include <etsi_its_cam_msgs/EmbarkationStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBOOLEAN.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::EmbarkationStatus convert_EmbarkationStatustoRos(const EmbarkationStatus_t& _EmbarkationStatus_in)
	{
		etsi_its_cam_msgs::EmbarkationStatus EmbarkationStatus_out;
		convert_BOOLEANtoRos(_EmbarkationStatus_in, EmbarkationStatus_out.value);
		return EmbarkationStatus_out;
	}
}