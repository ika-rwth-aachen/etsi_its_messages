#pragma once

#include <ReferencePosition.h>
#include <etsi_its_cam_msgs/ReferencePosition.h>
#include <convertLatitude.h>
#include <convertLongitude.h>
#include <convertPosConfidenceEllipse.h>
#include <convertAltitude.h>

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
}