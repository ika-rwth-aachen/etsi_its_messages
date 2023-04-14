#pragma once

#include <etsi_its_cam_coding/BasicContainer.h>
#include <etsi_its_cam_msgs/BasicContainer.h>
#include <etsi_its_cam_conversion/convertStationType.h>
#include <etsi_its_cam_conversion/convertReferencePosition.h>

namespace etsi_its_cam_conversion
{
	void convert_BasicContainertoRos(const BasicContainer_t& _BasicContainer_in, etsi_its_cam_msgs::BasicContainer& _BasicContainer_out)
	{
		convert_StationTypetoRos(_BasicContainer_in.stationType, _BasicContainer_out.stationType);
		convert_ReferencePositiontoRos(_BasicContainer_in.referencePosition, _BasicContainer_out.referencePosition);
	}
	void convert_BasicContainertoC(const etsi_its_cam_msgs::BasicContainer& _BasicContainer_in, BasicContainer_t& _BasicContainer_out)
	{
		memset(&_BasicContainer_out, 0, sizeof(BasicContainer_t));
		convert_StationTypetoC(_BasicContainer_in.stationType, _BasicContainer_out.stationType);
		convert_ReferencePositiontoC(_BasicContainer_in.referencePosition, _BasicContainer_out.referencePosition);
	}
}