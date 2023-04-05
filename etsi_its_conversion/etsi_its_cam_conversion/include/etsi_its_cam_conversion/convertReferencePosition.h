#pragma once

#include <etsi_its_cam_coding/ReferencePosition.h>
#include <etsi_its_cam_msgs/ReferencePosition.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertPosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertAltitude.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ReferencePosition convert_ReferencePositiontoRos(const ReferencePosition_t& _ReferencePosition_in)
	{
		etsi_its_cam_msgs::ReferencePosition ReferencePosition_out;
		ReferencePosition_out.latitude = convert_LatitudetoRos(_ReferencePosition_in.latitude);
		ReferencePosition_out.longitude = convert_LongitudetoRos(_ReferencePosition_in.longitude);
		ReferencePosition_out.positionConfidenceEllipse = convert_PosConfidenceEllipsetoRos(_ReferencePosition_in.positionConfidenceEllipse);
		ReferencePosition_out.altitude = convert_AltitudetoRos(_ReferencePosition_in.altitude);
		return ReferencePosition_out;
	}
	ReferencePosition_t convert_ReferencePositiontoC(const etsi_its_cam_msgs::ReferencePosition& _ReferencePosition_in)
	{
		ReferencePosition_t ReferencePosition_out;
		memset(&ReferencePosition_out, 0, sizeof(ReferencePosition_t));
		ReferencePosition_out.latitude = convert_LatitudetoC(_ReferencePosition_in.latitude);
		ReferencePosition_out.longitude = convert_LongitudetoC(_ReferencePosition_in.longitude);
		ReferencePosition_out.positionConfidenceEllipse = convert_PosConfidenceEllipsetoC(_ReferencePosition_in.positionConfidenceEllipse);
		ReferencePosition_out.altitude = convert_AltitudetoC(_ReferencePosition_in.altitude);
		return ReferencePosition_out;
	}
}