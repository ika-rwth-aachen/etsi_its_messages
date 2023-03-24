#pragma once

#include <etsi_its_cam_coding/CoopAwareness.h>
#include <etsi_its_cam_msgs/CoopAwareness.h>
#include <etsi_its_cam_conversion/convertGenerationDeltaTime.h>
#include <etsi_its_cam_conversion/convertCamParameters.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CoopAwareness convert_CoopAwarenesstoRos(const CoopAwareness_t& _CoopAwareness_in)
	{
		etsi_its_cam_msgs::CoopAwareness CoopAwareness_out;
		CoopAwareness_out.generationDeltaTime = convert_GenerationDeltaTimetoRos(_CoopAwareness_in.generationDeltaTime);
		CoopAwareness_out.camParameters = convert_CamParameterstoRos(_CoopAwareness_in.camParameters);
		return CoopAwareness_out;
	}
}