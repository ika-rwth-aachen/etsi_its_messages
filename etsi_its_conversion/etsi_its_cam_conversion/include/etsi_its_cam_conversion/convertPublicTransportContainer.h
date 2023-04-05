#pragma once

#include <etsi_its_cam_coding/PublicTransportContainer.h>
#include <etsi_its_cam_msgs/PublicTransportContainer.h>
#include <etsi_its_cam_conversion/convertEmbarkationStatus.h>
#include <etsi_its_cam_conversion/convertPtActivation.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PublicTransportContainer convert_PublicTransportContainertoRos(const PublicTransportContainer_t& _PublicTransportContainer_in)
	{
		etsi_its_cam_msgs::PublicTransportContainer PublicTransportContainer_out;
		PublicTransportContainer_out.embarkationStatus = convert_EmbarkationStatustoRos(_PublicTransportContainer_in.embarkationStatus);
		if(_PublicTransportContainer_in.ptActivation)
		{
			PublicTransportContainer_out.ptActivation = convert_PtActivationtoRos(*_PublicTransportContainer_in.ptActivation);
			PublicTransportContainer_out.ptActivation_isPresent = true;
		}
		return PublicTransportContainer_out;
	}
	PublicTransportContainer_t convert_PublicTransportContainertoC(const etsi_its_cam_msgs::PublicTransportContainer& _PublicTransportContainer_in)
	{
		PublicTransportContainer_t PublicTransportContainer_out;
		memset(&PublicTransportContainer_out, 0, sizeof(PublicTransportContainer_t));
		PublicTransportContainer_out.embarkationStatus = convert_EmbarkationStatustoC(_PublicTransportContainer_in.embarkationStatus);
		if(_PublicTransportContainer_in.ptActivation_isPresent)
		{
			auto ptActivation = convert_PtActivationtoC(_PublicTransportContainer_in.ptActivation);
			PublicTransportContainer_out.ptActivation = new PtActivation_t(ptActivation);
		}
		return PublicTransportContainer_out;
	}
}