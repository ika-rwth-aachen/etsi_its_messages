#pragma once

#include <etsi_its_cam_coding/DeltaReferencePosition.h>
#include <etsi_its_cam_msgs/DeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertDeltaLatitude.h>
#include <etsi_its_cam_conversion/convertDeltaLongitude.h>
#include <etsi_its_cam_conversion/convertDeltaAltitude.h>

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
	DeltaReferencePosition_t convert_DeltaReferencePositiontoC(const etsi_its_cam_msgs::DeltaReferencePosition& _DeltaReferencePosition_in)
	{
		DeltaReferencePosition_t DeltaReferencePosition_out;
		memset(&DeltaReferencePosition_out, 0, sizeof(DeltaReferencePosition_t));
		DeltaReferencePosition_out.deltaLatitude = convert_DeltaLatitudetoC(_DeltaReferencePosition_in.deltaLatitude);
		DeltaReferencePosition_out.deltaLongitude = convert_DeltaLongitudetoC(_DeltaReferencePosition_in.deltaLongitude);
		DeltaReferencePosition_out.deltaAltitude = convert_DeltaAltitudetoC(_DeltaReferencePosition_in.deltaAltitude);
		return DeltaReferencePosition_out;
	}
}