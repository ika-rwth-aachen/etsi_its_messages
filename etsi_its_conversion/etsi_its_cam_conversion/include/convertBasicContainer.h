#pragma once

#include <BasicContainer.h>
#include <etsi_its_cam_msgs/BasicContainer.h>
#include <convertStationType.h>
#include <convertReferencePosition.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::BasicContainer convert_BasicContainertoRos(const BasicContainer_t& _BasicContainer_in)
	{
		etsi_its_cam_msgs::BasicContainer BasicContainer_out;
		BasicContainer_out.stationType = convert_StationTypetoRos(_BasicContainer_in.stationType);
		BasicContainer_out.referencePosition = convert_ReferencePositiontoRos(_BasicContainer_in.referencePosition);
		return BasicContainer_out;
	}
}