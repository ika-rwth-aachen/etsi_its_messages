#pragma once

#include <DeltaReferencePosition.h>
#include <etsi_its_cam_msgs/DeltaReferencePosition.h>
#include <convertDeltaLatitude.h>
#include <convertDeltaLongitude.h>
#include <convertDeltaAltitude.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DeltaReferencePosition convert_DeltaReferencePositiontoRos(const DeltaReferencePosition_t& _DeltaReferencePosition_in)
	{
		etsi_its_cam_msgs::DeltaReferencePosition DeltaReferencePosition_out;
		DeltaReferencePosition_out.deltaLatitude = convert_DeltaLatitudetoRos(_DeltaReferencePosition_in.deltaLatitude);
		DeltaReferencePosition_out.deltaLongitude = convert_DeltaLongitudetoRos(_DeltaReferencePosition_in.deltaLongitude);
		DeltaReferencePosition_out.deltaAltitude = convert_DeltaAltitudetoRos(_DeltaReferencePosition_in.deltaAltitude);
		return DeltaReferencePosition_out;
	}
}