#pragma once

#include <PublicTransportContainer.h>
#include <etsi_its_cam_msgs/PublicTransportContainer.h>
#include <convertEmbarkationStatus.h>
#include <convertPtActivation.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PublicTransportContainer convert_PublicTransportContainertoRos(const PublicTransportContainer_t& _PublicTransportContainer_in)
	{
		etsi_its_cam_msgs::PublicTransportContainer PublicTransportContainer_out;
		PublicTransportContainer_out.embarkationStatus = convert_EmbarkationStatustoRos(_PublicTransportContainer_in.embarkationStatus);
		if(_PublicTransportContainer_in.ptActivation)
		{
			PublicTransportContainer_out.ptActivation = convert_PtActivationtoRos(*_PublicTransportContainer_in.ptActivation);
		}
		return PublicTransportContainer_out;
	}
}